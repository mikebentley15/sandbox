from matplotlib import (
    pyplot as plt,
    style
)
from collections import deque
import time
from math import sin, pi

from simple_drawnow import drawnow

style.use('fivethirtyeight')

window_size = 10_000 # milliseconds
sample_rate = 33 # milliseconds

data = deque(maxlen=window_size // sample_rate)
times = deque(maxlen=window_size // sample_rate)
def animate_plot(new_xy, axis):
    timestamp, val = new_xy
    times.append(timestamp)
    if len(times) > 2:
        print(times[-1] - times[-2])
    data.append(val)
    axis.clear()
    axis.plot(times, data)

def sine_generator(amplitude, period):
    start = time.monotonic()
    while True:
        delta = time.monotonic() - start
        val = amplitude * sin(2 * pi * (delta % period) / period)
        yield (delta, val)

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

amplitude = 3.5
period = 4.3

with plt.ion():
    start_time = time.monotonic()
    for xy in sine_generator(amplitude, period):
        interval = sample_rate / 1000
        next_time = start_time + interval

        animate_plot(xy, ax1)
        plt.show()

        current_time = time.monotonic()
        delta = next_time - current_time
        if delta > 0:
            #time.sleep(delta)
            plt.pause(delta)
        start_time = current_time

#ani = animation.FuncAnimation(fig, animate_plot,
#        frames=sine_generator(amplitude, period),
#        cache_frame_data=False,
#        blit=False, # for True, would need to return iterable of artists
#        interval=2, #sample_rate, # I think extra sleeps are being put in
#        fargs=(ax1,))
#plt.show()
