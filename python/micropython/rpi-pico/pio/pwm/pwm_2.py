# Very simple example that creates a PWM signal

from utime import sleep_ms
from machine import Pin
import rp2

# (2 instructions)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_fastest():
    wrap_target()
    set(pins, 1)  # 1 cycle
    set(pins, 0)  # 1 cycle
    wrap()        # 0 cycles

# (3 instructions)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_fast():
    label('again')
    set(pins, 1)  # 1 cycle
    set(pins, 0)  # 1 cycle
    jmp('again')  # 1 cycle

# (3 instructions)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_with_waits():
   wrap_target()
   set(pins, 1) [2] # 3 cycles
   set(pins, 0) [2] # 3 cycles
   wrap()

# (2 instructions)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_slow_1():
    wrap_target()
    set(pins, 1) [31] # 32 cycles
    set(pins, 0) [31] # 32 cycles
    wrap()

# (4 instructions)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_slow_2():
    wrap_target()
    set(pins, 1) [31] # 32 cycles
    nop()        [31] # 32 cycles
    set(pins, 0) [31] # 32 cycles
    nop()        [31] # 32 cycles
    wrap()

# not sure why this didn't work...
# - now I do, cannot use set() with a number larger than 31.
# - it's a problem that the python implementation silently ignores these
#   parsing errors.
#
# (9 instructions)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_slow_3():
    # goal: to set pins to on for 5,000 cycles, then off for 5,000 cycles

    # loop forever
    wrap_target()
    set(y, 1)               # 1 cycle

    label('toggle_and_sleep')

    # toggle
    mov(pins, y)            # 1 cycle

    # delay for 4,996 cycles
    set(x, 151)    [12]     # 13 cycles
    label('sleep_inner')
    nop()          [31]     # 32 * 151 cycles = 4,832 cycles
    jmp(x_dec, 'sleep_inner') # 1 * 151 cycles = 151 cycles

    # if done, got to the end
    jmp(not_y, 'end')       # 1 cycle

    # else, set y to off and go back to toggle and sleep
    set(y, 0)               # 1 cycle
    jmp('toggle_and_sleep') # 1 cycle

    label('end')
    nop()                   # 1 cycle (to finish it off)
    wrap()                  # 0 cycles

# Doesn't work for the same reason as blink_slow_3()
#
# (8 instructions)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_slow_4():
    # goal: to set pins to on for 5,000 cycles, then off for 5,000 cycles

    # loop forever
    wrap_target()

    # turn on
    set(pins, 1)            # 1 cycle

    # delay for 4,999 cycles
    set(x, 150)    [15]     # 16 cycles
    label('sleep_1')
    nop()          [31]     # 32 * 151 cycles = 4,832 cycles
    jmp(x_dec, 'sleep_1')   # 1 * 151 cycles = 151 cycles

    # turn off
    set(pins, 0)            # 1 cycle

    # delay for 4,999 cycles
    set(x, 150)    [15]     # 16 cycles
    label('sleep_2')
    nop()          [31]     # 32 * 151 cycles = 4,832 cycles
    jmp(x_dec, 'sleep_2')   # 1 * 151 cycles = 151 cycles

    wrap()                  # 0 cycles

# (14 instructions)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_slow_5():
    # goal: to set pins to on for 5,000 cycles, then off for 5,000 cycles

    # loop forever
    wrap_target()

    # turn on
    set(pins, 1)                 # 1 cycle

    # delay for 4,999 cycles = 7 + 32 * (32 + 32 + 32 + 32 + 28)
    set(x, 31)             [6]   # 7 cycles
    label('sleep_1')
    nop()                 [31]  # 32 * 32
    nop()                 [31]  # 32 * 32
    nop()                 [31]  # 32 * 32
    nop()                 [31]  # 32 * 32
    jmp(x_dec, 'sleep_1') [27]  # 32 * 28

    # turn off
    set(pins, 0)                # 1 cycle

    # delay for 4,999 cycles = 7 + 32 * (32 + 32 + 32 + 32 + 28)
    set(x, 31)             [6]   # 7 cycles
    label('sleep_2')
    nop()                 [31]  # 32 * 32
    nop()                 [31]  # 32 * 32
    nop()                 [31]  # 32 * 32
    nop()                 [31]  # 32 * 32
    jmp(x_dec, 'sleep_2') [27]  # 32 * 28

    wrap()                      # 0 cycles

# specify the period between pulses at the beginning from the fifo
# queue with put() in units of 10's of cycles (a zero given has a
# period of 10 cycles, a one given has a period of 20 cycles, etc.)
# The number of cycles is equal to 10*(1 + (put value)).
#
# Note: the pulse is high for 8 cycles, and will be low for the
#   remainder, so keep that in mind.  For a value of 0 in the put
#   queue, a pulse is given every 10 cycles, so this means 8 on and
#   2 off.  For a value of 1 into the queue, it is 8 on and 12 off.
#
# (8 instructions)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def pulse_period():
    # x represents the amount of 10's of cycles to sleep
    # beyond the 10 cycles of sending the pulse
    set(x, 0)           # 1 cycle: initialize x

    wrap_target()

    label("nosleep")

    # pulse with width = 8 cycles
    set(pins, 1)            # 1 cycle: on

    # pull from TX queue.  If empty, pulls from x value by default
    # (because of the noblock), which is perfect since that's our
    # storage of the value in question.
    pull(noblock)           # 1 cycle: pull from queue (or x) into osr
    mov(x, osr)      [4]    # 5 cycles: update x value
    mov(y, x)               # 1 cycle: copy x into y for use

    # have low for 2 + 10*y cycles
    set(pins, 0)            # 1 cycles: off, was on for 8 cycles

    # sleep in a loop for 10*y + 1 cycles
    label('sleep')
    jmp(not_y, 'nosleep')       # 1 cycle
    jmp(y_dec, 'sleep')    [8]  # 9 cycles: loop

    wrap()

# specify the period between pulses at the beginning from the fifo
# queue with put() in units of cycles (a zero given has a
# period of 100 cycles, a one given has a period of 101 cycles, etc.)
# The number of cycles is equal to 100 + (put value).
#
# Note: the pulse is high for 50 cycles, and will be low for the
#   remainder, so keep that in mind.  For a value of 0 in the put
#   queue, a pulse is given every 100 cycles, so this means 50 on and
#   50 off.  For a value of 1 into the queue, it is 50 on and 51 off.
#
# (7 instructions)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def pulse_period_1MHz():
    # x represents the amount of cycles to sleep
    # beyond the base amount of 100 cycles
    set(x, 0)           # 1 cycle: initialize x

    wrap_target()

    # pulse with width = 50 cycles = 32 + 18
    set(pins, 1)    [31]    # 32 cycles: on

    # pull from TX queue.  If empty, pulls from x value by default
    # (because of the noblock), which is perfect since that's our
    # storage of the value in question.
    pull(noblock)   [17]    # 18 cycles: pull from queue (or x) into osr

    # have low for 50 + y cycles = 32 + 16 + 1 + (y + 1)
    set(pins, 0)    [31]    # 32 cycles: off
    mov(x, osr)     [15]    # 16 cycles: update x value
    mov(y, x)               # 1 cycle: copy x into y for use

    # sleep in a loop for (y + 1) cycles
    label('sleep')
    jmp(y_dec, 'sleep')     # 1 cycle: loop

    wrap()

# specify the period between pulses at the beginning from the fifo
# queue with put() in units of cycles (a zero given has a
# period of 200 cycles, a one given has a period of 201 cycles, etc.)
# The number of cycles is equal to 200 + (put value).
#
# Note: the pulse is high for 100 cycles, and will be low for the
#   remainder, so keep that in mind.  For a value of 0 in the put
#   queue, a pulse is given every 200 cycles, so this means 100 on and
#   100 off.  For a value of 1 into the queue, it is 100 on and 101 off.
#
# (10 instructions)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def pulse_period_2MHz():
    # x represents the amount of cycles to sleep
    # beyond the base amount of 200 cycles

    # 2 cycles of setup
    set(x, 0)               # 1 cycle: initialize osr, to be copied into x
    mov(osr, x)             # 1 cycle: initialize osr, to be copied into x

    wrap_target()

    # pulse with width = 100 cycles = 32*3 + 4
    set(pins, 1)    [31]    # 32 cycles: on

    # pull from TX queue.  If empty, pulls from x value by default
    # (because of the noblock), which is perfect since that's our
    # storage of the value in question.
    mov(x, osr)     [31]    # 32 cycles: restore x value
    pull(noblock)   [31]    # 32 cycles: pull from queue (or x) into osr
    mov(x, osr)      [3]    #  4 cycles: update x value

    # have low for 100 + x cycles
    set(pins, 0)    [31]    # 32 cycles: off, was on for 100 cycles
    nop()           [31]    # 32 cycles: sleep
    nop()           [31]    # 32 cycles: sleep
    nop()            [2]    #  3 cycles: sleep

    # sleep in a loop for x + 1 cycles
    label('sleep')
    jmp(x_dec, 'sleep')     # 1 cycle: loop

    wrap()


# specify the period between pulses at the beginning from the fifo
# queue with put() in units of cycles (a zero given has a
# period of 12,500 cycles, a one given has a period of 12,501 cycles,
# etc.).  The number of cycles is equal to 12,500 + (put value).
#
# Note: the pulse is high for 6,250 cycles, and will be low for the
#   remainder, so keep that in mind.  For a value of 0 in the put
#   queue, a pulse is given every 12,5000 cycles, so this means 6,250 on and
#   6,250 off.  For a value of 1 into the queue, it is 6,250 on and 6,251 off.
#
# (20 instructions)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def pulse_period_125MHz():
    # x represents the amount of cycles to sleep
    # beyond the base amount of 12,500 cycles

    # 2 cycles of setup
    set(x, 0)               # 1 cycle: initialize osr, to be copied into x
    mov(osr, x)             # 1 cycle

    # For mov sleeping user input, both x and osr are required.
    # this leaves us y and isr as two scratch spaces to use for the
    # constant sleeping periods

    wrap_target()

    # pulse with width = 6,250 cycles = 10 + 32*(3 + 32*6)
    set(pins, 1)            # 1 cycle: high
    set(y, 31)       [7]    # 8 cycles: sleeping loop variable
    label('loop_high_y')

    # pull from TX queue.  If empty, pulls from x value by default
    # (because of the noblock), which is perfect since that's our
    # storage of the value in question.
    mov(x, osr)     [31]    # 32 * 32 cycles: restore x value
    pull(noblock)   [31]    # 32 * 32 cycles: pull from queue (or x) into osr
    mov(x, osr)     [31]    # 32 * 32 cycles: update x value
    nop()           [31]    # 32 * 32 cycles: sleep
    nop()           [31]    # 32 * 32 cycles: sleep
    nop()           [31]    # 32 * 32 cycles: sleep

    jmp(y_dec, 'loop_high_y') [2] # 32 * 3 cycles

    set(y, 31)              #  1 cycle: reset sleeping loop variable

    # have low for 6,250 + x cycles = 6 * 32 * 32 + 3 * 32 + 9 + (x+1)
    label('loop_low_y')
    set(pins, 0)    [31]    # 32 * 32 cycles: low, was high for 6,250 cycles
    nop()           [31]    # 32 * 32 cycles: sleep
    nop()           [31]    # 32 * 32 cycles: sleep
    nop()           [31]    # 32 * 32 cycles: sleep
    nop()           [31]    # 32 * 32 cycles: sleep
    nop()           [31]    # 32 * 32 cycles: sleep
    jmp(y_dec, 'loop_low_y') [2] # 32 * 3 cycles

    nop()            [8]    # 9 cycles: sleep

    # sleep in a loop for x + 1 cycles
    label('sleep')
    jmp(x_dec, 'sleep')     # 1 cycle: loop

    wrap()


# This PIO routine sends pulses such that if the clock speed is at 125 MHz,
# then the pulse width of the on is 1 microsecond (125 cycles) and the
# pulse width of the off position is at least 1 microsecond (125 cycles)
# plus the number of additional cycles given by the TX FIFO queue.
#
# This method only differs from pulse_period_125MHz() by the minimum
# width of the on and off periods from 6,250 cycles down to 125 cycles.
# This allows for pulses at most at 500 kHz (from 10 kHz acheivable
# from pulse_period_125MHz()), but with much control to achieve any
# additional integer number of cycles for the low region.
#
# (10 instructions)
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def pulse_period_125MHz_fast():
    # x represents the amount of cycles to sleep
    # beyond the base amount of 250 cycles per pulse

    # 2 cycles of setup
    set(x, 0)               # 1 cycle: initialize osr, to be copied into x
    mov(osr, x)             # 1 cycle

    wrap_target()

    # pulse with width = 125 cycles = 3*32 + 29
    set(pins, 1)    [31]    # 32 cycles: on

    # pull from TX queue.  If empty, pulls from x value by default
    # (because of the noblock), which is perfect since that's our
    # storage of the value in question.
    mov(x, osr)     [31]    # 32 cycles: restore x value
    pull(noblock)   [31]    # 32 cycles: pull from queue (or x) into osr
    mov(x, osr)     [28]    # 29 cycles: update x value

    # have low for 125 + x cycles = 3*32 + 28 + (x+1)
    set(pins, 0)    [31]    # 32 cycles: off, was on for 100 cycles
    nop()           [31]    # 32 cycles: sleep
    nop()           [31]    # 32 cycles: sleep
    nop()           [27]    # 28 cycles: sleep

    # sleep in a loop for x + 1 cycles
    label('sleep')
    jmp(x_dec, 'sleep')     # 1 cycle: loop

    wrap()

# Measure the number of pulses (a low followed by a high).  This count is
# pushed to the RX FIFO queue and zeroed every X*3 clock cycles.  It is assumed
# that X is sufficiently large, such as 125 million/3 (i.e., 41,666,667).
#
# If the frequency is set to the maximum of 125 MHz, then setting X to
# 41,666,667 means a report every one second with a sampling frequency of
# once every 3 cycles (or 41,666,667 Hz).  This means this routine has a maximum
# detection of frequencies at 20,833,333 Hz (or about 20 MHz) which is awesome!
#
# The way to set this value is by preloading the TX FIFO queue with a 32-bit
# value before starting this state machine.  If no such value is in the TX FIFO,
# this routine blocks until there is.
#
# (17 instructions)
@rp2.asm_pio()
def pulse_measure_3cycle():
    pull(block)
    mov(y, invert(null))   # set y to maximum 0xffffffff (counting is by decrementing)

    # x is the timeout in 3x cycles
    # - this means that jmp(x--, 'restart') needs to be every third cycle
    # isr is the count of identified pulses

    # when restarting, it takes 6 cycles before checking for high
    # other than that, checks for high, then low, every 3 cycles :)

    label('restart')
    mov(x, osr)            # initialize x to the timeout
    mov(isr, invert(y))    # copy opposite of y into isr
    jmp(x_dec, 'skip_1')
    jmp('restart')
    label('skip_1')
    push(noblock)          # push and zero isr
    mov(y, invert(null))   # set y to maximum 0xffffffff (counting is by decrementing)

    wrap_target()

    # wait for low (checks every 3 cycles)
    label('wait_for_low')
    jmp(x_dec, 'skip_2')
    jmp('restart')
    label('skip_2')
    jmp(pin, 'wait_for_low') [1] # 2 cycles: while high, keep waiting

    # wait for high (checks every 3 cycles)
    label('wait_for_high')
    jmp(x_dec, 'skip_3')
    jmp('restart')
    label('skip_3')
    jmp(pin, 'exit_wait_for_high') # once high, goto the exit
    jmp('wait_for_high')           # else, wait
    label('exit_wait_for_high')

    # record the value
    jmp(y_dec, 'wait_for_low') # decrement y and wrap around anyway

    wrap()
    


# default frequency is 125 MHz.
# To generate a 12.5 Khz signal, we need to divide the frequency by 10,000
# In my application, I want the maximum of probably 9.6 kHz

p0 = Pin(0, Pin.OUT)
p1 = Pin(1, Pin.IN)
p2 = Pin(2, Pin.OUT)
p3 = Pin(3, Pin.IN)

# for RP2040, fastest my arduino mega can measure is about 62 kHz.
# This is achieved around a frequency of 187,000
#
# In this experiment, I've connected pin 0 to pin 1 and pin 2 to pin 3
# I want to see how well four state machines all communicate together on the
# same clock and how accurate the pulses and measurements are.
sm0 = rp2.StateMachine(0, pulse_period_125MHz_fast, set_base=p0)
sm1 = rp2.StateMachine(1, pulse_measure_3cycle, jmp_pin=p1)
#sm2 = rp2.StateMachine(2, pulse_period_125MHz_fast, set_base=p2)
#sm3 = rp2.StateMachine(3, pulse_measure_3cycle, jmp_pin=p3)

# initialize state machines with parameters
#sm0.put(0) # run it at its fastest or 500 kHz
#sm0.put((125_000_000 // 10_000) - 250) # should run at 10,000 Hz
sm0.put((125_000_000 // 312_500) - 250) # should run at 312,500 Hz
sm1.put(round(125_000_000 / 3)) # report once per second

#sm2.put((125_000_000 // 31_250) - 250) # should run at 312,500 Hz
#sm3.put(round(125_000_000 / 3)) # report once per second

# activate state machines
sm0.active(1)
sm1.active(1)
#sm2.active(1)
#sm3.active(1)

def print_latest_readings(state_machines):
    # prints only if there are readings to read from all sms
    # empties each queue and only takes the last value present
    if all(x.rx_fifo() > 0 for x in state_machines):
        readings = []
        for sm in state_machines:
            while sm.rx_fifo() > 0:
                r = sm.get()
            readings.append(r)
        for r in readings:
            print(f'    {r} Hz', end='')
        print()

def set_new_freq(sm_gen, freq, sm_read=None):
    # sets the new generator frequency
    # resets the reading sm to have an empty RX queue
    sm_gen.put((125_000_000 // freq) - 250)
    while sm_read.rx_fifo() > 0:
        sm_read.get()