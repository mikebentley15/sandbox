from linked_list import SinglyLinkedList as SLL

import time
from collections import deque

class timewith():
    def __init__(self, name=''):
        self.name = name
        self.start = time.time()

    @property
    def elapsed(self):
        return time.time() - self.start

    def checkpoint(self, name=''):
        print('{timer} {checkpoint} took {elapsed} seconds'.format(
            timer=self.name,
            checkpoint=name,
            elapsed=self.elapsed,
            ))

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.checkpoint('finished')
        pass

def time_prepend():
    n = 50000
    print('time_prepend,   n =', n)
    with timewith('  linked list  '):
        container = SLL()
        for x in range(n):
            container.prepend(x)
        del container
    with timewith('  list         '):
        container = list()
        for x in range(n):
            container.insert(0, x)
        del container
    with timewith('  deque        '):
        container = deque()
        for x in range(n):
            container.appendleft(x)
        del container

def time_construct():
    n = 50000
    print('time_construct, n =', n)
    with timewith('  linked list  '): SLL(range(n))
    with timewith('  list         '): list(range(n))
    with timewith('  deque        '): deque(range(n))

def time_append():
    n = 5000
    print('time_append,    n =', n)
    with timewith('  linked list  '):
        container = SLL()
        for x in range(n):
            container.append(x)
    del container
    with timewith('  list         '):
        container = list()
        for x in range(n):
            container.append(x)
    del container
    with timewith('  deque        '):
        container = deque()
        for x in range(n):
            container.append(x)
    del container

def time_iterate():
    n = 500000
    print('time_iterate,   n =', n)
    container = SLL(range(n))
    with timewith('  linked list  '):
        for x in container:
            pass
    del container
    container = list(range(n))
    with timewith('  list         '):
        for x in container:
            pass
    del container
    container = deque(range(n))
    with timewith('  deque        '):
        for x in container:
            pass
    del container

def time_extend():
    n = 500000
    print('time_extend,    n =', n)
    container = SLL(range(n))
    with timewith('  linked list  '):
        container.extend(range(n))
    del container
    container = list(range(n))
    with timewith('  list         '):
        container.extend(range(n))
    del container
    container = deque(range(n))
    with timewith('  deque        '):
        container.extend(range(n))
    del container

def time_pop():
    n = 50000
    print('time_pop,       n =', n)
    container = SLL(range(n))
    with timewith('  linked list  '):
        for _ in range(n):
            container.pop()
    del container
    container = list(range(n))
    with timewith('  list         '):
        for _ in range(n):
            container[0]
            del container[0]
    del container
    container = deque(range(n))
    with timewith('  deque        '):
        for _ in range(n):
            container.pop()
    del container

def main():
    time_prepend()
    time_construct()
    time_append()
    time_iterate()
    time_extend()
    time_pop()

if __name__ == '__main__':
    main()
