from machine import enable_irq, disable_irq, idle, Pin
from micropython import const
import rp2
import time

GAIN_32  = const(2)
GAIN_64  = const(3)
GAIN_128 = const(1)

class HX711_gpio:
    def __init__(self, clock_pin, data_pin, gain=GAIN_128):
        self.clock_pin = clock_pin
        self.data_pin = data_pin
        self._gain = gain
        self.power_up()

    def gain(self):
        return self._gain

    def set_gain(self, gain):
        self._gain = gain
        self.read()

    def read(self):
        # wait for the device to be ready
        for _ in range(500):
            if self.data_pin.value() == 0:
                break
            time.sleep_ms(1)
        else:
            raise OSError('Sensor does not respond')

        # shift in data, and gain & channel info
        result = 0
        for j in range(24 + self._gain):
            state = disable_irq()
            self.clock_pin.on()
            self.clock_pin.off()
            enable_irq(state)
            result = (result << 1) | self.data_pin.value()

        # shift back the extra bits
        result >>= self._gain

        # convert from signed 24-bit to unsigned 24-bit
        result = result ^ 0x800000

        return result

    def power_up(self):
        #self.clock_pin.high()
        self.clock_pin.low()
        self.set_gain(self._gain)

    def power_down(self):
        self.clock_pin.high()

def callback(hx, rawval):
    print(time.ticks_ms(), hex(rawval))

def main():
    data = Pin(14, Pin.IN, pull=Pin.PULL_DOWN)
    clock = Pin(15, Pin.OUT)

    h = HX711_gpio(clock, data)
    for _ in range(10): # read for one second
        callback(h, h.read())

if __name__ == '__main__':
    main()
