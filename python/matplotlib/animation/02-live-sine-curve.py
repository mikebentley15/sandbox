from matplotlib import (
    pyplot as plt,
    animation,
    style
)
from collections import deque
import time
from math import sin, pi

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

line = None
def animate_plot_2(new_xy, axis):
    global line
    timestamp, val = new_xy
    times.append(timestamp)
    if len(times) > 2:
        print(times[-1] - times[-2])
    data.append(val)
    if line is None:
        line, = axis.plot(times, data)
    else:
        line.set_xdata(times)
        line.set_ydata(data)

        datamin = min(data)
        datamax = max(data)
        datarange = datamax - datamin

        axis.set_ylim(datamin - 0.1*datarange, datamax + 0.1*datarange)
        latest_time = times[-1]
        axis.set_xbound(latest_time - window_size / 1000, latest_time)

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

ani = animation.FuncAnimation(fig, animate_plot_2,
        frames=sine_generator(amplitude, period),
        cache_frame_data=False,
        blit=False, # for True, would need to return iterable of artists
        interval=sample_rate, #sample_rate, # I think extra sleeps are being put in
        fargs=(ax1,))
plt.show()
