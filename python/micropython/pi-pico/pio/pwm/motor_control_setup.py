# Configurable example of creating and measuring a PWM signal of arbitrary
# frequency

from micropython import const
from utime import sleep_ms
from machine import Pin
import rp2


# =========
# constants
# =========

# generic for state machines
SM_FREQ            = const(125_000_000)

# for generating (i.e., running pulse_period_125MHz_fast)
PULSE_GEN_BASE_CYCLES = const(250)

# microstepping constants
MICROSTEP_1        = const( 1)
MICROSTEP_2        = const( 2)
MICROSTEP_4        = const( 4)
MICROSTEP_8        = const( 8)
MICROSTEP_16       = const(16)

# motor conversions
MOTOR_MICROSTEP         = const(MICROSTEP_16)
FULL_STEPS_PER_ROTATION = const(200) # full steps / rot
STEPS_PER_ROTATION      = const(MOTOR_MICROSTEP * FULL_STEPS_PER_ROTATION) # steps/rot
DEGREES_PER_ROTATION    = const(360)  # deg/rot
LINEAR_PITCH_MM         = const(5)    # mm/rot

# pins

ROTARY_DIR_PIN_NUM  = const(16)
ROTARY_STEP_PIN_NUM = const(17)
ROTARY_MS1_PIN_NUM  = const(20)
ROTARY_MS2_PIN_NUM  = const(19)
ROTARY_MS3_PIN_NUM  = const(18)

LINEAR_DIR_PIN_NUM  = const(21)
LINEAR_STEP_PIN_NUM = const(22)
LINEAR_MS1_PIN_NUM  = const(28)
LINEAR_MS2_PIN_NUM  = const(27)
LINEAR_MS3_PIN_NUM  = const(26)

rdir = Pin(ROTARY_DIR_PIN_NUM,  Pin.OUT)
rstp = Pin(ROTARY_STEP_PIN_NUM, Pin.OUT)
rms1 = Pin(ROTARY_MS1_PIN_NUM,  Pin.OUT)
rms2 = Pin(ROTARY_MS2_PIN_NUM,  Pin.OUT)
rms3 = Pin(ROTARY_MS3_PIN_NUM,  Pin.OUT)

ldir = Pin(LINEAR_DIR_PIN_NUM,  Pin.OUT)
lstp = Pin(LINEAR_STEP_PIN_NUM, Pin.OUT)
lms1 = Pin(LINEAR_MS1_PIN_NUM,  Pin.OUT)
lms2 = Pin(LINEAR_MS2_PIN_NUM,  Pin.OUT)
lms3 = Pin(LINEAR_MS3_PIN_NUM,  Pin.OUT)

# state machines (uninitialized)
# Note: they are on the same PIO block 0 (SM 0-3), so they share the same
# instruction space of 32 instructions and they share the same base clock
rmot_sm = rp2.StateMachine(0)
lmot_sm = rp2.StateMachine(1)


# ================
# helper functions
# ================

def init():
    set_microstep()
    initialize_motor_state_machines()
    rdir.high()
    ldir.high()

def set_microstep():
    '''
    Set the pin outputs for both motors for the microstepping choice in the
    constant MOTOR_MICROSTEP.
    '''
    _set_to_microstep(MOTOR_MICROSTEP, rms1, rms2, rms3)
    _set_to_microstep(MOTOR_MICROSTEP, lms1, lms2, lms3)

def initialize_motor_state_machines():
    '''
    Initializes rmot_sm and lmot_sm global state machines

    Simply sets their machine code and assignemd GPIO pins.  Does not start the
    state machines.
    '''
    rmot_sm.init(pulse_period_125MHz_fast, set_base=rstp)
    lmot_sm.init(pulse_period_125MHz_fast, set_base=lstp)

def set_freq(sm: rp2.StateMachine, freq: int) -> None:
    '''
    sets the new generator frequency for a state machine running
    pulse_period_125MHz_fast().  Will automatically handle starting and
    stopping the state machine based on the requested frequency (e.g., will
    stop the state machine if 0 or a negative number is passed as the desired
    frequency).

    For motors, this frequency is the steps per second.  You can convert
    degrees per second to steps per second with degrees_to_steps().  Likewise,
    you can convert millimeters per second to steps per second with
    millimeters_to_steps().
    '''

    # note: negative frequencies are truncated to zero
    if freq <= 0:
        # turn off the state machine to stop pulses
        sm.active(0)
    else:
        sm.put(round(SM_FREQ / freq) - PULSE_GEN_BASE_CYCLES)
        sm.active(1) # be sure to turn on if it's not running

def stop():
    rmot_sm.active(0)
    lmot_sm.active(0)

# right now, we only support moving forward.  If you want to change direction,
# you change the direction pin yourself.
def set_rot_speed(revs_per_second):
    set_freq(rmot_sm, revs_per_second * STEPS_PER_ROTATION)

def set_lin_speed(mm_per_second):
    set_freq(lmot_sm, millimeters_to_steps(mm_per_second))

def toggle_rot_direction():
    rdir.toggle()

def toggle_lin_direction():
    ldir.toggle()

def degrees_to_steps(degrees):
    return degrees * STEPS_PER_ROTATION / DEGREES_PER_ROTATION

def millimeters_to_steps(mm):
    return mm * STEPS_PER_ROTATION / LINEAR_PITCH_MM

def _set_to_microstep(choice, ms1, ms2, ms3):
    def set_pins(v1, v2, v3):
        ms1.value(v1)
        ms2.value(v2)
        ms3.value(v3)
    if   choice == MICROSTEP_1 :  set_pins(0, 0, 0)
    elif choice == MICROSTEP_2 :  set_pins(1, 0, 0)
    elif choice == MICROSTEP_4 :  set_pins(0, 1, 0)
    elif choice == MICROSTEP_8 :  set_pins(1, 1, 0)
    elif choice == MICROSTEP_16:  set_pins(1, 1, 1)


# ===========================
# state machine assembly code
# ===========================

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def pulse_period_125MHz_fast():
    '''
    This PIO routine sends pulses such that if the clock speed is at 125 MHz,
    then the pulse width of the on is 1 microsecond (125 cycles) and the
    pulse width of the off position is at least 1 microsecond (125 cycles)
    plus the number of additional cycles given by the TX FIFO queue.
    
    This method only differs from pulse_period_125MHz() by the minimum
    width of the on and off periods from 6,250 cycles down to 125 cycles.
    This allows for pulses at most at 500 kHz (from 10 kHz acheivable
    from pulse_period_125MHz()), but with much control to achieve any
    additional integer number of cycles for the low region.
    
    (10 instructions)
    '''
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
