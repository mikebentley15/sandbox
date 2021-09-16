from machine import Pin
from micropython import const
import rp2
import time

GAIN_32  = const(2)
GAIN_64  = const(3)
GAIN_128 = const(1)

class HX711_cb:
    '''
    Specifies how to communicate with the HX711 chip predominantly used with
    strain guage force sensors.

    This version allows a user to register a callback function (hence the _cb
    suffix to the class name) that takes two arguments, the HX711 object and
    the most recent raw reading.  You can get the most recent value from
    self.value and nanosecond timestamp from self.time_ns.

    Example:

        data = Pin(14, Pin.IN, pull=Pin.PULL_DOWN)
        clock = Pin(15, Pin.OUT)
        sm = rp2.StateMachine(0)
        h = HX711_cb(clock, data, sm, cb=(lambda hx, val: print(hex(val))))
        h.power_up()
        # at this point, the callback will be called with interrupts

    Public attributes:
    - clock_pin: the clock pin given in __init__().  Do not change
    - data_pin: the data pin given in __init__().  Do not change
    - sm: the state machine given in __init__().  Do not change
    - gain: the gain given in __init__().  Changing this causes the state
      machine to restart.
    - cb: the callback function given in __init__().  You can change this.
    - value: last raw unsigned value read from the sensor.  Set by this class's
      interrupt callback.  Initialized to -1, which is an invalid value.
    - ticks_us: relative time of the last reading (from time.ticks_us()) of
      when this class's interrupt callback was called.  Initialized to -1,
      which is an invalid value.
      Note: this value does wrap around roughly every 71 minutes.
    '''

    def __init__(self, clock_pin, data_pin, sm, gain=GAIN_128, cb=None):
        '''
        @param clock_pin (machine.Pin): serial clock pin, initialized as an output
            pin.   This pin is used for signalling data retrieval as well as
            signaling the HX711 chip to go to sleep.
        @param data_pin (machine.Pin): data pin, initialized as a pull down input
            pin (e.g., data = Pin(12, Pin.IN, pull=Pin.PULL_DOWN)).  This pin
            is used to read data based on the usage of the clock_pin pin.
        @param sm (rp2.StateMachine): one of the available state machines.  The
            RP2040 has up to 8 of them on two separate PIO's (PIO 0 has state
            machines 0-3, PIO 1 has state machines 4-7).  This does not need to
            be initialized, and if it is, it will be replaced. For example,
            just do rp2.StateMachine(1) to specify state machine 1 here.  This
            class will initialize it beyond that.
        @param gain (int): one of GAIN_32, GAIN_64, or GAIN_128.  See the data
            sheet for more information.  The basic intuition is that if you
            need to measure small signals, go with high gain, which is usually
            the case for force sensors.
            Default is a gain of 128
        @param cb (callable taking raw reading): callback function that will be
            called from the interrupt handler of when the new value comes in
            from the sensor.  Will be sent this HX711 object and the most
            recent raw reading.
        '''
        self.clock_pin = clock_pin
        self.data_pin = data_pin
        self.sm = sm
        self._gain = gain
        self.cb = cb

        self.value = -1
        self.ticks_us = -1

        # create the state machine
        self.sm.init(self._hx711_pio, freq=2_000_000,
                     sideset_base=self.clock_pin,
                     in_base=self.data_pin,
                     jmp_pin=self.data_pin)

        # register callback
        self.sm.irq(self._irq_cb)

    @property
    def gain(self):
        return self._gain

    @gain.setter
    def set_gain(self, gain):
        self._gain = gain
        self.power_down()
        self.power_up()

    def power_down(self):
        '''
        Stop the state machine and signal the chip to power down

        Takes 60 microseconds after calling this function for it to be fully
        powered down which uses much less power (not that it uses much before).
        '''
        self.sm.active(0) # stop the state machine
        self.clock_pin.low()
        self.clock_pin.high()

    def power_up(self):
        '''
        Signal the chip to power up and start the state machine

        Takes one full cycle of readings to ensure the proper gain setting is
        in place before readings start in earnest.
        '''
        self.clock_pin.low()
        # give the state machine how many bits to expect
        self.sm.restart() # restart from the beginning
        self.sm.put(self._gain + 24 - 1)  # set pulse count 25-27
        self.sm.active(1) # start the state machine

    def _irq_cb(self, sm):
        'Record this latest value and call the callback if set'
        # discard the gain bits
        self.ticks_us = time.ticks_us() # capture the current relative time
        val = sm.get() >> self._gain  # discard the extra gain bits
        val ^= 0x800000               # convert signed 24-bit to unsigned
        self.value = val              # cache the value in this object
        self.cb(self, val)            # call the callback

    @rp2.asm_pio(
        sideset_init=rp2.PIO.OUT_LOW,
        in_shiftdir=rp2.PIO.SHIFT_LEFT,
        autopull=False,
        autopush=False,
    )
    def _hx711_pio():
        '''
        State machine code for the PIO to communicate and retrieve data from
        the HX711 chip.

        Protocol:
        - Data is ready to read when the data line goes low.
        - We control the clock.
        - Before reading, we set the gain by the number of pulses sent each
          iteration.  25 for 128 gain, 26 for 32 gain, and 27 for 64 gain.
        - For every sent clock pulse (which needs to be between 0.2us to 50us),
          the data line will go high or low indicating that bit from most
          significant bit first MSB.
        - There are 24 bits to read each cycle (so 24 pulses with data to
          read), but we need to ensure we continue pulsing to the 25, 26, or 27
          pulses for the desired current gain.

        Using this routine:
        - push the number of bits to receive minus one (24 for 25 pulses, 25
          for 26 pulses, or 26 for 27 pulses) to the TX fifo queue before
          starting this routine.  It is only read once.  If you want to change
          the number of bits to receive, stop the routine, restart it, push the
          new number of bits, and activate it again.
        - each reading will be pushed to the RX fifo queue, then this machine's
          IRQ signal will be flagged, which if there is a registered callback,
          it will be called after the queue has been populated.
        - Make sure to promptly remove from the queue, either by checking the
          queue regularly, or by removing from the queue from the IRQ callback.
          If it gets full, then new readings will be dropped and the queue will
          have stale data.

        (10 instructions)
        '''
        pull(block)           .side(0)     # get the number of bits to retrieve
        mov(x, osr)           .side(0)     # store it in x

        # generate x+1 pulses once to initialize the gain and channel of HX711
        wait(0, pin, 0)       .side(0)     # wait for data line to go low
        label('initloop')
        nop()                 .side(1) [1] # 2 cycles high
        jmp(x_dec, 'initloop').side(0) [1] # 2 cycles low

        # main loop
        wrap_target()                      # loop forever

        mov(x, osr)           .side(0)     # reset x back to # clock cycles

        # Wait for data to go high, then low again, indicating new data
        # Note: instead of waiting for the pin to go high, we could sleep a set
        #   amount, however the datasheet does not specify how long that is.
        wait(1, pin, 0)       .side(0)     # wait for data line to go high
        wait(0, pin, 0)       .side(0)     # wait for data line to go low
        

        label('bitloop')
        nop()                 .side(1) [1] # activate edge
        in_(pins, 1)          .side(0)     # get the pin and shift it in
        jmp(x_dec, 'bitloop') .side(0)     # test for more bits

        push(noblock)         .side(0)     # deliver data and start over
        irq(noblock, rel(0))  .side(0)     # signal host new data is ready

        wrap()                             # loop forever

class HX711(HX711_cb):
    '''
    Adds additional functionality to HX711_cb.  This version also allows the
    user to provide a callback, but this callback will only get the HX711
    object, no value.  That is because this class has many different notions of
    value.  So simply use the appropriate getter function from that callback to
    get what you want.

    This class also keeps a certain history of values.

    The additional functionality in this class gives the ability to
    - tare: store and use an offset to the zero point
    - scaled: convert raw readings into a meaningful number like kg or g
    - average: return the average of the last X readings, where X is less than
      or equal to the history size.
    - wait: wait for X new readings to come in

    Additional public properties:
    - count (read-only): number of readings so far from interrupt callbacks
    - offset: zero point.  Can be set directly, or by calling self.tare()
    - scale: divisor for raw readings to convert to your units of choice.  This
      is to be set by the user through a calibration of taring, then putting a
      known force on the sensor and tweaking until the units come out as
      expected.
    '''

    def __init__(self, clock_pin, data_pin, sm, gain=GAIN_128, cb=None,
                 history_size=10):
        '''
        Inputs that are the same as the parent object HX711_cb
        - clock_pin
        - data_pin
        - sm
        - gain
        - cb

        New parameters:
        @param history_size (int): number of elements to keep in a ring buffer.
            This is used to run tare(), read_average(), etc.  Instead of
            waiting to read the next X values, simply use the history.  If you
            want the functionality of waiting for the next X values, call
            self.wait(X).
        '''
        super().__init__(clock_pin, data_pin, sm, gain=gain, ch=ch)

        self._count = 0 # number of readings so far
        self.offset = 0 # zero point
        self.scale  = 1 # scaling 

    @property
    def count(self):
        return self._count

def callback(hx, rawval):
    print(time.ticks_ms(), hex(rawval))

def main():
    data = Pin(14, Pin.IN, pull=Pin.PULL_DOWN)
    clock = Pin(15, Pin.OUT)
    sm = rp2.StateMachine(4)
    h = HX711_cb(clock, data, sm, cb=callback)
    h.power_up()

    # sleep so that we don't have a prompt.  Instead, CTRL-C  will stop the callbacks
    try:
        while True:
            time.sleep_ms(100)
    except KeyboardInterrupt:
        h.power_down()

if __name__ == '__main__':
    main()
