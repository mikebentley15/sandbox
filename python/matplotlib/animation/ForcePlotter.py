import time
import multiprocessing as mp

def _plot_worker(connection):
    from matplotlib import (
        pyplot as plt,
        style
    )
    from collections import deque
    from sched import scheduler
    import time
    import enum

    style.use('fivethirtyeight')

    # unlimited size.  We will prune based on timestamps
    data = deque()
    times = deque()

    LATEST_TIME = 0
    LATEST_VAL = 1
    WINDOW_SIZE = 2 # seconds
    POLL_TIME = 3 # time between polls for data
    state = [None, None, 5, 0.02]
    def try_recv_all(blocking=False):
        '''
        Parses entire connection queue, returning True if there is something
        new to plot
        '''
        have_something_new = False
        while connection.poll() or (blocking and not have_something_new):
            update_type, update_value = connection.recv() # blocking
            if update_type == 'window_size':
                state[WINDOW_SIZE] = update_value
            elif update_type == 'min_plot_time':
                state[POLL_TIME] = update_value
            elif update_type == 'xy':
                if have_something_new:
                    data.append(state[LATEST_VAL])
                    times.append(state[LATEST_TIME])
                state[LATEST_TIME], state[LATEST_VAL] = update_value
                #print('received new value:', update_value)
                have_something_new = True
            else:
                print('Warning: unrecognized update type -', update_type)
        return have_something_new

    # block until first value is received
    try_recv_all(blocking=True)

    time_offset = time.monotonic() - state[LATEST_TIME] # synchronize my time
    get_time = lambda: time.monotonic() - time_offset

    fig = plt.figure()
    axis = fig.add_subplot(1, 1, 1)
    line, = axis.plot(times, data)  # empty plot

    def update_data():
        data.append(state[LATEST_VAL])
        times.append(state[LATEST_TIME])

        # prune to the window size
        cutoff = state[LATEST_TIME] - state[WINDOW_SIZE]
        while times[0] < cutoff:
            data.popleft()
            times.popleft()

    def update_xbound():
        current_time = get_time()
        axis.set_xbound(current_time - state[WINDOW_SIZE], current_time)

    def update_axis():
        line.set_xdata(times)
        line.set_ydata(data)
        update_xbound()
        if len(data) > 2:
            datamin = min(data)
            datamax = max(data)
            datarange = datamax - datamin
            axis.set_ylim(datamin - datarange/10, datamax + datarange/10)

    with plt.ion(): # interactive mode - non-blocking matplotlib
        print('showing plot')
        plt.show()
        while plt.get_fignums(): # while the figure is not closed
            while not try_recv_all(): # until there is a new value, service the fig
                update_xbound()
                fig.canvas.start_event_loop(state[POLL_TIME])

            # update the figure - handle all queued events
            update_data()
            update_axis()
            fig.canvas.flush_events()

    print('exiting plotting process - user closed the window')

_ctx = mp.get_context('spawn')

class ForcePlotter(_ctx.Process):
    def __init__(self, window_size=10, min_plot_time=0.02):
        '''
        Plots forces versus time on a moving plot running in a separate process
        '''
        recv_pipe, send_pipe = _ctx.Pipe(duplex=False)
        super().__init__(target=_plot_worker, args=(recv_pipe,))
        self._window_size = window_size
        self._min_plot_time = min_plot_time
        self._send_pipe = send_pipe
        self._child_pipe = recv_pipe
        self._creation_time = time.monotonic()

    @property
    def window_size(self):
        return self._window_size

    @window_size.setter
    def window_size(self, value):
        self._window_size = value
        self._send_pipe.send(('window_size', value))

    @property
    def min_plot_time(self):
        return self._min_plot_time

    @min_plot_time.setter
    def min_plot_time(self, value):
        self._min_plot_time = value
        self._send_pipe.send(('min_plot_time', value))

    def time_since_creation(self):
        return time.monotonic() - self._creation_time

    def append(self, value, timestamp=None):
        if not timestamp:
            timestamp = self.time_since_creation()
        self._send_pipe.send(('xy', (timestamp, value)))
