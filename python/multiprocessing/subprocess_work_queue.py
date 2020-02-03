#!/usr/bin/env python3
'''
This script attempts to use a work queue available to the children processes of
a pool.
'''

import multiprocessing as mp
import queue
import time
from functools import partial
from contextlib import closing

class QueuePool:
    def __init__(self, work_q):
        self.work_q = work_q
        self.return_q = mp.SimpleQueue()

    def map(func, iterable, chunksize=None):
        self.work_q.put((self.return_q, func, iterable))
        return self.return_q.get()

def factorial(n):
    if n <= 1: return 1
    return n * factorial(n-1)

def worker(q_pool, value):
    mapper = q_pool.map
    return mapper(factorial, [value] * 10)

def main():
    with closing(mp.Pool()) as work_pool, \
         closing(mp.Pool()) as job_pool, \
         closing(mp.Queue()) as work_q:

        async_result = job_pool.map_async(
            worker, [(QueuePool(work_q), x) for x in [1000, 5000, 10000]])
        while not async_result.ready():
            try:
                return_q, func, iterable = work_q.get_nowait()
            except queue.Empty:
                time.sleep(0.1)
            else:
                return_q.put(work_pool.map(func, iterable))

        result = async_result.get()

    for value in result:
        print(value)

if __name__ == '__main__':
    main()
