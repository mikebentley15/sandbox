'''
Same as 05, but this time, try to update the data from a separate thread rather
than using a generator.
'''

import time
from math import sin, pi
import multiprocessing as mp
import random

# process worker functions

def plot_worker(connection):
    from matplotlib import (
        pyplot as plt,
        style
    )
    from collections import deque
    from sched import scheduler
    style.use('fivethirtyeight')

    window_size = 5 # seconds
    sample_rate = 0.030 # seconds

    data = deque(maxlen=int(window_size / sample_rate))
    times = deque(maxlen=int(window_size / sample_rate))

    latest_val = connection.recv() # block until first value is received
    fig = plt.figure()
    axis = fig.add_subplot(1, 1, 1)
    line, = axis.plot(times, data)  # empty plot

    def update_axis():
        line.set_xdata(times)
        line.set_ydata(data)

        if len(data) > 2:
            datamin = min(data)
            datamax = max(data)
            datarange = datamax - datamin
            latest_time = times[-1]

            axis.set_ylim(datamin - datarange/10, datamax + datarange/10)
            axis.set_xbound(latest_time - window_size, latest_time)

    base_time = time.monotonic()
    get_time = lambda: time.monotonic() - base_time
    start_time = time.monotonic()
    with plt.ion(): # interactive mode - non-blocking matplotlib
        plt.show()
        while plt.get_fignums(): # while the figure is not closed
            next_time = start_time + sample_rate
            while connection.poll():  # see if there is a new value
                latest_val = connection.recv()
            #latest_val = sine_func(3.5, 2.3)
            # here we will duplicate stale data for the sake of interactive
            # plotting
            data.append(latest_val)
            times.append(start_time - base_time)

            # update the figure - handle all queued events
            update_axis()
            fig.canvas.flush_events()

            # sleep until next update is needed
            sleep_until(next_time)
            start_time = next_time
    print('exiting plotting process - user closed the window')

def data_generation_worker(connection):
    amplitude = 3.5
    period = 2.3
    while True:
        #time.sleep(random.randint(3, 400) / 1000) # random sleep interval
        time.sleep(3e-3)
        connection.send(sine_func(amplitude, period))

# helper functions

def sine_func(amplitude, period):
    return amplitude * sin(2 * pi * (time.monotonic() % period) / period)

def sleep_until(future_time):
    'sleep until future time as returned from time.monotonic()'
    remaining_time = future_time - time.monotonic()
    if remaining_time > 0:
        time.sleep(remaining_time)

def main():
    precv, psend = mp.Pipe(duplex=False)

    child = mp.Process(target=plot_worker, args=(precv,))
    child.start()

    try:
        data_generation_worker(psend)
    except KeyboardInterrupt:
        print('Stopped from Ctrl-C')
        pass
    except:
        raise
    finally:
        child.kill()
        child.join()

if __name__ == '__main__':
    main()
