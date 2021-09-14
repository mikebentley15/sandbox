# Configurable example of creating and measuring a PWM signal of arbitrary
# frequency

from micropython import const
from utime import sleep_ms
from machine import Pin
import rp2


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
sm2 = rp2.StateMachine(2, pulse_period_125MHz_fast, set_base=p2)
sm3 = rp2.StateMachine(3, pulse_measure_3cycle, jmp_pin=p3)

# =========
# constants
# =========

# generic for state machines
sm_freq            = const(125_000_000)

# for measuring (i.e., running pulse_measure_3cycle)
sm_read_sample_ms  = const(100)
sm_read_check_freq = sm_freq / 3
sm_read_arg        = round(sm_read_check_freq * sm_read_sample_ms / 1000)

# for generating (i.e., running pulse_period_125MHz_fast)
pulse_gen_base_cycles = 250

# initialize state machines with parameters
sm1.put(sm_read_arg)
sm3.put(sm_read_arg)

#sm2.put((125_000_000 // 31_250) - 250) # should run at 312,500 Hz
#sm3.put(round(125_000_000 / 3)) # report once per second

# activate state machines
sm1.active(1)
sm3.active(1)

def clear_rx_fifo(sm: rp2.StateMachine):
    # clears the RX fifo queue of a statemachine
    while sm.rx_fifo():
        sm.get()

def get_next_reading(*statemachines, count=1):
    # clear all queued readings and then wait and return the next one
    # can give any number of state machines to this function
    # if no state machines are given, returns None and does nothing
    # if only one state machine is given, return just one reading
    # otherwise, return a list of readings.
    if not statemachines:
        return None
    for sm in statemachines:
        clear_rx_fifo(sm)
    # do a blocking call to get() count times
    readings = [0] * len(statemachines)
    for _ in range(count):
        for i in range(len(statemachines)):
            readings[i] += statemachines[i].get()
    readings = [x/count for x in readings]
    if len(statemachines) == 1:
        return readings[0]
    return readings

def ensure_list(x):
    if isinstance(x, list):
        return x
    return [x]

def sample2freq(samples):
    if isinstance(samples, list):
        return [sample2freq(x) for x in samples] # recursive
    return 1000 * samples / sm_read_sample_ms

def print_next_freq_reading(*statemachines, count=1):
    readings = ensure_list(get_next_reading(*statemachines, count=count))
    for r in readings:
        print(f'    {sample2freq(r)} Hz', end='')
    print()

def print_freq_running_avg(*statemachines, count=1):
    totals = [0] * len(statemachines)
    for i in range(count):
        current = ensure_list(get_next_reading(*statemachines, count=1))
        for j in range(len(statemachines)):
            totals[j] += current[j]
        for r in totals:
            print(f'    {sample2freq(r/(i+1))} Hz', end='')
        print()

def set_freq(sm: rp2.StateMachine, freq: int) -> None:
    # sets the new generator frequency for a state machine running
    # pulse_period_125MHz_fast()

    # note: negative frequencies are truncated to zero
    if freq <= 0:
        # turn off the state machine to stop pulses
        sm.active(0)
    else:
        sm.put(round(sm_freq / freq) - pulse_gen_base_cycles)
        sm.active(1) # be sure to turn on if it's not running

