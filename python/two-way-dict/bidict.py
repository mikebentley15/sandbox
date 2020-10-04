'''
Implements the TwoWayMapping immutable class and TwoWayDict mutable class that
maintain both mappings of key to value and value to key.
'''

from collections.abc import Reversible, Mapping, MutableMapping

class TwoWayMapping(Reversible, Mapping):
    '''
    An immutable dictionary that gives bidirectionality.  That is a mapping
    both from key to value (like a normal dictionary) and a mapping from value
    to key.  Therefore, we cannot have duplicate values just like we cannot
    have duplicate keys.

    It is immutable for simplicity and ease of implementation.

    In addition to normal dictionary accessors and calling reversed() on
    instances of this class, we have the following:

    - value(self, key): return the value from the key (same as self[key])
    - key(self, value): return the key from the value

    Note: calling reversed() on an instance of this class will reverse the
    key-value relationship and is extremely efficient.

    >>> TwoWayMapping()
    TwoWayMapping{}
    >>> m = TwoWayMapping({1: 2, 2: 3, 3: 4})
    >>> m[2]
    3
    >>> m[3]
    4
    >>> m.key(2)
    1
    >>> m.value(2)
    3
    >>> m.key(3)
    2
    >>> m.value(3)
    4
    >>> m == {1: 2, 2: 3, 3: 4}
    True
    >>> r = reversed(m)
    >>> r == {2: 1, 3: 2, 4: 3}
    True
    >>> r[2]
    1
    >>> r[3]
    2
    >>> r.key(2)
    3
    >>> r.value(2)
    1
    >>> r.key(3)
    4
    >>> r.value(3)
    2
    '''

    def __init__(self, items=()):
        '''
        Initializes the mapping.  Will raise a ValueError if duplicate values
        are given.  But if duplicate keys are given, then later keys will
        overwrite earlier keys.
        '''
        self._data = dict(items)
        self._inverse = dict((v, k) for (k, v) in self._data.items())
        if len(self._data) != len(self._inverse):
            raise ValueError('Duplicate keys or values are not allowed')

    def key(self, value):
        'Return the key associated with the given value'
        return self._inverse[value]

    def value(self, key):
        'Return the value associated with the given key'
        return self._data[key]

    def __reversed__(self):
        'Return a shallow copy that reverses the key-value relationship'
        other = TwoWayMapping()
        other._data = self._inverse
        other._inverse = self._data
        return other

    def __getitem__(self, key):
        return self._data[key]

    def __iter__(self):
        return iter(self._data)

    def __len__(self):
        return len(self._data)

    def __repr__(self):
        return 'TwoWayMapping{}'.format(self._data)

class TwoWayDict(TwoWayMapping, MutableMapping):
    '''
    Adds mutability to a TwoWayMapping class.

    Note: there is some wierdness that goes on.  For example, by setting a
    value, you may actually make the container smaller because that value was
    already set to another key.   They way this is resolved is that the mapping
    to the previous key is removed before setting the requested mapping.

    >>> TwoWayDict()
    TwoWayDict{}
    >>> m = TwoWayDict({1: 2, 2: 3, 3: 4})
    >>> m[2]
    3
    >>> m[3]
    4
    >>> m.key(2)
    1
    >>> m.value(2)
    3
    >>> m.key(3)
    2
    >>> m.value(3)
    4
    >>> m[3] = 6
    >>> m.key(6)
    3
    >>> m.value(3)
    6
    >>> m == {1: 2, 2: 3, 3: 4}
    False
    >>> m == {1: 2, 2: 3, 3: 6}
    True
    >>> r = reversed(m)
    >>> r == {2: 1, 3: 2, 6: 3}
    True
    >>> r[2]
    1
    >>> r[3]
    2
    >>> r.key(2)
    3
    >>> r.value(2)
    1
    >>> r.key(3)
    4
    >>> r.value(3)
    2
    >>> r[3] = 1
    >>> r == {3: 1, 6: 3}
    True
    >>> m == {1: 3, 3: 6}
    True
    '''

    def __setitem__(self, key, value):
        if self[key] is value:
            return
        if key in self._data:
            del self._inverse[self._data[key]]
        if value in self._inverse:
            del self._data[self._inverse[value]]
        self._data[key] = value
        self._inverse[value] = key

    def __delitem__(self, key):
        value = self._data.pop(key)
        self._inverse.pop(value)

    def __repr__(self):
        return 'TwoWayDict{}'.format(self._data)
