'''
Same as 04, but this time, try to update the line rather than clearing and
redrawing the axis
'''

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
sample_rate = 10 # milliseconds

line = None

data = deque(maxlen=window_size // sample_rate)
times = deque(maxlen=window_size // sample_rate)
def animate_plot(new_xy, axis):
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
        # TODO: update axis

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
    plt.show()
    start_time = time.monotonic()
    interval = sample_rate / 1000
    for xy in sine_generator(amplitude, period):
        next_time = start_time + interval

        animate_plot(xy, ax1)
        # do either
        # - fig.canvas.flush_events() - blocks until all events are handled
        # - fig.canvas.start_event_loop(delta) - blocks until delta time even
        #   if there are still events to process
        fig.canvas.flush_events()
        #plt.show()
        #plt.draw_all()

        current_time = time.monotonic()
        delta = next_time - current_time
        if delta > 0:
            time.sleep(delta)
            #plt.pause(delta)
            #fig.canvas.start_event_loop(delta)
        start_time = next_time

        if not plt.get_fignums():
            print('User closed the window, goodbye!')
            break

#ani = animation.FuncAnimation(fig, animate_plot,
#        frames=sine_generator(amplitude, period),
#        cache_frame_data=False,
#        blit=False, # for True, would need to return iterable of artists
#        interval=2, #sample_rate, # I think extra sleeps are being put in
#        fargs=(ax1,))
#plt.show()
