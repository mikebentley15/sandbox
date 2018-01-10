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
pool = multiprocessing.Pool()
out = pool.map(range, range(0, 10))
print('Output:   ', out)
print('Flattened:', list(flatten(out)))
