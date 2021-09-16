from machine import Pin
from micropython import const
import rp2
import time

GAIN_32  = const(2)
GAIN_64  = const(3)
GAIN_128 = const(1)

class HX711_pio_dumb:
    'from https://github.com/robert-hh/hx711/blob/master/hx711_pio.py'
    def __init__(self, clock_pin, data_pin, gain=GAIN_128):
        self.clock_pin = clock_pin
        self.data_pin = data_pin
        self._gain = gain
        self.sm = rp2.StateMachine(0, self._hx711_pio, freq=1_000_000,
                                   sideset_base=self.clock_pin,
                                   in_base=self.data_pin,
                                   jmp_pin=self.data_pin)
        self.power_up()

    def gain(self):
        return self._gain

    def set_gain(self, gain):
        self._gain = gain
        self.read()

    def read(self):
        # Feed the waiting state machine & get the data
        self.sm.active(1)
        self.sm.put(250_000) # wait time of 500ms
        self.sm.put(self._gain + 24 - 1)
        result = self.sm.get() >> self._gain
        self.sm.active(0)
        if result == 0x7fff_ffff:
            raise OSError('Sensor does not respond')
        result = result ^ 0x80_0000
        return result

    def power_up(self):
        #self.clock_pin.high()
        self.clock_pin.low()
        self.set_gain(self._gain)

    def power_down(self):
        self.clock_pin.high()

    @rp2.asm_pio(
        sideset_init=rp2.PIO.OUT_LOW,
        in_shiftdir=rp2.PIO.SHIFT_LEFT,
        autopull=False,
        autopush=False,
    )
    def _hx711_pio():
        pull(block)         .side (0)   # get the initial wait time
        mov(y, osr)         .side (0)
        pull(block)         .side (0)   # get the number of clock cycles
        mov(x, osr)         .side (0)

        wait(0, pin, 0)     .side (0)   # wait for in pin to be low

        label("start")
        jmp(pin, "nostart") .side (0)   # not ready yet
        jmp("bitloop")      .side (0)   # ready, get data

        label("nostart")
        jmp(y_dec, "start") .side (0)   # another attempt
        mov(isr, y)         .side (0)   # set 0xffffffff as error value
        jmp("finish")       .side (0)

        label("bitloop")
        nop()               .side (1)   # active edge
        in_(pins, 1)        .side (1)   # get the pin and shift it in
        jmp(x_dec, "bitloop")  .side (0)   # test for more bits
        
        label("finish")
        push(block)         .side (0)   # no, deliver data and start over

def callback(hx, rawval):
    print(time.ticks_ms(), hex(rawval))

def main():
    data = Pin(14, Pin.IN, pull=Pin.PULL_DOWN)
    clock = Pin(15, Pin.OUT)

    h = HX711_pio_dumb(clock, data)
    for _ in range(10): # read for one second
        callback(h, h.read())

if __name__ == '__main__':
    main()
