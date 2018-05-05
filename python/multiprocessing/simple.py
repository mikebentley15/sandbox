#!/usr/bin/env python

from __future__ import print_function
import multiprocessing
import collections

#flatten = lambda lst: [y for x in lst for y in x]
def flatten(lst):
   for item in lst:
      if isinstance(item, collections.Iterable):
         for elem in flatten(item):
            yield elem
      else:
         yield item
with multiprocessing.Pool(5) as pool:
    out = pool.map(range, range(0, 10))
print('Output:   ', out)
print('Flattened:', list(flatten(out)))

def f1(x):
    return x+1
with multiprocessing.Pool(5) as pool:
    out = pool.map(f1, range(10))
print('Output:    ', out)

def f2(x,y):
    return x+1+y+2
def f3(x):
    return f2(x[0],x[1])
with multiprocessing.Pool(5) as pool:
    # These do not work:
    #out = pool.map(f2, zip(range(10), range(10)))
    #out = pool.map(lambda x: f2(x[0],x[1]), zip(range(10), range(10)))
    # This does
    out = pool.map(f3, zip(range(10), range(10)))
print('Output:    ', out)

import sys
prog = 'hi'
def f4(x):
    return sys.argv
with multiprocessing.Pool(5) as pool:
    out = pool.map(f4, range(10))
print('Output:    ', out)

def f5(x):
    return x,x+1
with multiprocessing.Pool(5) as pool:
    out = pool.map(f5, range(10))
print('Output:    ', out)

import multiprocessing as mp

workqueue = mp.Queue()
for i in range(10): workqueue.put(i)
resultqueue = mp.Queue()
def lazy_worker(inq, outq):
    import time
    import queue
    try:
        while True:
            time.sleep(0.1)
            outq.put(inq.get(False))
    except queue.Empty:
        pass
workers = []
for i in range(3):
    p = mp.Process(target=lazy_worker, args=(workqueue,resultqueue))
    p.start()
    workers.append(p)
for i in range(10):
    print(resultqueue.get())
for w in workers:
    w.join()
