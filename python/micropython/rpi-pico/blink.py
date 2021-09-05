from machine import Pin, ADC
from utime import sleep, ticks_us

def blink():
    led = Pin(25, Pin.OUT)
    with open('times.txt', 'a') as out:

        def pulse(duration):
            print(f'{ticks_us()}: led on', file=out)
            led.on()
            sleep(duration)
            print(f'{ticks_us()}: led off', file=out)
            led.off()
            sleep(duration)

        def long_pulse(): pulse(0.3)
        def short_pulse(): pulse(0.2)

        for i in range(3): short_pulse()
        for i in range(3): long_pulse()
        for i in range(3): short_pulse()

def main(): blink()

if __name__ == '__main__':
    main()
