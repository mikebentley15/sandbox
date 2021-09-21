'''
Same as 07, but the logic is behind a separate class now
'''

from ForcePlotter import ForcePlotter

import time
from math import sin, pi
import random
import numpy as np
import itertools

BASE_TIME = time.monotonic()
def get_time():
    return time.monotonic() - BASE_TIME

def data_generation_worker(callback):
    amplitude = 3.5
    period = 2.3
    while True:
        time.sleep(random.randint(3, 50) / 1000) # random sleep interval
        #time.sleep(3e-3)
        value = sine_func(amplitude, period)
        current_time = get_time()
        callback((current_time, value))

# helper functions

def sine_func(amplitude, period):
    return amplitude * sin(2 * pi * (get_time() % period) / period)

def sleep_until(future_time):
    'sleep until future time as returned from get_time()'
    remaining_time = future_time - get_time()
    if remaining_time > 0:
        time.sleep(remaining_time)

def main():
    plot = ForcePlotter()
    plot.start()

    A = 3.5 # amplitude
    P = 2.3 # period

    def print_state(t, val):
        print(f'A: {A:.2f}, P: {P:.2f}, W: {plot.window_size:.2f}, U: {plot.min_plot_time:.2f}, t: {t:.2f}, val: {val:.2f}')

    interval = 0.03
    try:
        for A, P, U, W in itertools.product(np.linspace(1,4,7),
                                            np.linspace(1,4,7),
                                            np.linspace(0.01, 0.1, 10),
                                            np.linspace(10, 1, 200)):
            plot.min_plot_time = U
            plot.window_size = W
            val = sine_func(A, P)
            t = plot.time_since_creation()
            print_state(t, val)
            plot.append(val, t)
            time.sleep(interval)

            if not plot.is_alive():
                print('identified that the plot process has exited')
                break
    except KeyboardInterrupt:
        print('Stopped from Ctrl-C')
        pass
    except:
        raise
    finally:
        plot.terminate()
        plot.join()

if __name__ == '__main__':
    main()
