'''
Same as 06, but this time, plot data as fast as it comes in
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
    #sample_rate = 0.030 # seconds

    # unlimited size.  We will prune based on timestamps
    data = deque()
    times = deque()

    latest_time, latest_val = connection.recv() # block until first value is received
    fig = plt.figure()
    axis = fig.add_subplot(1, 1, 1)
    line, = axis.plot(times, data)  # empty plot

    def update_data():
        data.append(latest_val)
        times.append(latest_time)

        # prune to the window size
        cutoff = latest_time - window_size
        while times[0] < cutoff:
            data.popleft()
            times.popleft()

    def update_xbound():
        current_time = get_time()
        axis.set_xbound(current_time - window_size, current_time)

    def update_axis():
        line.set_xdata(times)
        line.set_ydata(data)
        update_xbound()
        if len(data) > 2:
            datamin = min(data)
            datamax = max(data)
            datarange = datamax - datamin
            axis.set_ylim(datamin - datarange/10, datamax + datarange/10)

    poll_time = 0.02 # time between polls for data
    with plt.ion(): # interactive mode - non-blocking matplotlib
        plt.show()
        while plt.get_fignums(): # while the figure is not closed
            while not connection.poll(): # until there is a new value, service the fig
                update_xbound()
                fig.canvas.start_event_loop(poll_time)

            # update the figure - handle all queued events
            while connection.poll(): # clear queue and take only the latest
                latest_time, latest_val = connection.recv()
            update_data()
            update_axis()
            fig.canvas.flush_events()

    print('exiting plotting process - user closed the window')

BASE_TIME = time.monotonic()
def get_time():
    return time.monotonic() - BASE_TIME

def data_generation_worker(connection):
    amplitude = 3.5
    period = 2.3
    while True:
        time.sleep(random.randint(3, 50) / 1000) # random sleep interval
        #time.sleep(3e-3)
        value = sine_func(amplitude, period)
        current_time = get_time()
        connection.send((current_time, value))

# helper functions

def sine_func(amplitude, period):
    return amplitude * sin(2 * pi * (get_time() % period) / period)

def sleep_until(future_time):
    'sleep until future time as returned from get_time()'
    remaining_time = future_time - get_time()
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
