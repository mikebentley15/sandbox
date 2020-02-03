#!/usr/bin/env python3
'''
This script explores the possability of having a pool in the child process
separate from a pool in the parent process.

According to

  https://stackoverflow.com/questions/17223301/python-multiprocessing-is-it-possible-to-have-a-pool-inside-of-a-pool'

this is not allowed because the Pool processes are implemented as daemons to
make sure they are cleaned up afterwards.  But daemon processes are not allowed
to have children.

That post demonstrates a workaround that is captured here.
'''


import multiprocessing
import multiprocessing.pool
from contextlib import closing

class NoDaemonProcess(multiprocessing.Process):
    @property
    def daemon(self):
        return False

    @daemon.setter
    def daemon(self, value):
        pass

class NoDaemonContext(type(multiprocessing.get_context())):
    Process = NoDaemonProcess

# We sub-class multiprocessing.pool.Pool instead of multiprocessing.Pool
# because the latter is only a wrapper function, not a proper class.
class MyPool(multiprocessing.pool.Pool):
    def __init__(self, *args, **kwargs):
        kwargs['context'] = NoDaemonContext()
        super(MyPool, self).__init__(*args, **kwargs)

def subsub(x):
    return x

def sub(limit):
    # a normal pool
    with closing(multiprocessing.Pool(processes=2)) as pool:
        return pool.map(subsub, range(limit+1))

def main():
    # using my pool without daemons
    with closing(MyPool(processes=2)) as pool:
        vals = pool.map(sub, range(10))
    for val in vals:
        print(val)

if __name__ == '__main__':
    main()
