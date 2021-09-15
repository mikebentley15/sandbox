# Very simple example that creates a PWM signal

import time
from machine import Pin
import rp2
from pulse_period import pulse_period

pin = Pin(17)
pin.init(Pin.OUT)

# default frequency is 125 MHz.
# To generate a 12.5 Khz signal, we need to divide the frequency by 10,000
# In my application, I want the maximum of probably 9.6 kHz

# for RP2040, fastest my arduino mega can measure is about 62 kHz.
# This is achieved around a frequency of 187,000
sm = rp2.StateMachine(0, pulse_period, freq=10000, set_base=pin)

sm.active(1)
