# Very simple example that creates a PWM signal

import time
from machine import Pin
import rp2

pin1 = Pin(17)
pin2 = Pin(22)

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def blink_fastest():
    label("loop")
    set(pins, 1)
    set(pins, 0)
    jmp("loop")

# for RP2040, fastest my arduino mega can measure is about 62 kHz.
# This is achieved around a frequency of 187,000
sm1 = rp2.StateMachine(0, blink_fastest, freq=187000, set_base=pin1)
sm2 = rp2.StateMachine(1, blink_fastest, freq=187000, set_base=pin2)

sm1.active(1)
sm2.active(1)

led = Pin(25)
led.init(Pin.OUT)
while True:
    led.toggle()
    time.sleep_ms(100)