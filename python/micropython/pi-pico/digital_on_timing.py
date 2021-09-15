from machine import Pin
from utime import ticks_us

print()

pin = Pin(10, Pin.OUT)
n = 1000


start = ticks_us()
for i in range(n):
    pass
end = ticks_us()
diff = end - start

print(f'Loop without setting a pin: {diff} us')

start = ticks_us()
for i in range(n):
    pin.on()
end = ticks_us()
diff = end - start

print(f'Total time for {n} digital writes to high: {diff} us')


start = ticks_us()
for i in range(n):
    pin.off()
end = ticks_us()
diff = end - start

print(f'Total time for {n} digital writes to low: {diff} us')


start = ticks_us()
for i in range(n):
    pin.toggle()
end = ticks_us()
diff = end - start

print(f'Total time for {n} digital toggles: {diff} us')


print()