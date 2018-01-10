'''
The definition of a SinglyLinkedList class
'''

class SinglyLinkedList(object):
    '''
    A singly-linked-list meaning we only have links forward and not backward.
    '''

    class _Node(object):
        'Private node class containing values and links'
        def __init__(self, value, next=None):
            self._value = value
            self._next = next

    def __init__(self, iterable=None):
        '''
        Initialize the list, optionally with initial elements
        '''
        self._head = None
        self._size = 0
        if iterable is not None:
            self.extend(iterable)

    def __len__(self):
        '''
        Return the number of elements in the list

        >>> s = SinglyLinkedList()
        >>> len(s)
        0
        >>> s.insert('a', 0)
        >>> len(s)
        1
        '''
        return self._size

    def _nodes_at(self, idx):
        'Returns (prev, current) where current is at index idx'
        prev = None
        current = self._head
        for _ in range(idx):
            prev = current
            current = current._next
        return (prev, current)

    def _insert_at(self, value, prev_node, next_node):
        '''
        Implementation of insertion at a location, and return the inserted node
        '''
        new_node = self._Node(value, next_node)
        if prev_node is None:
            self._head = new_node
        else:
            prev_node._next = new_node
        self._size += 1
        return new_node

    def _remove_node(self, prev_node, to_remove):
        'Implementation of removing a node from the list'
        if prev_node == None:
            self._head = to_remove._next
        else:
            prev_node._next = to_remove._next
        self._size -= 1

    def insert(self, value, idx):
        '''
        Insert value at index idx, shifting everything else to the right

        >>> s = SinglyLinkedList()
        >>> s.insert('a', 0)
        >>> s.insert('b', 1)
        >>> s.insert('c', 1)
        >>> len(s)
        3
        >>> s
        [a -> c -> b]
        >>> s.insert('d', -1)
        Traceback (most recent call last):
        ...
        IndexError: Index out of bounds: -1
        >>> s.insert('d', 4)
        Traceback (most recent call last):
        ...
        IndexError: Index out of bounds: 4
        >>> s.insert('d', 3)
        >>> s
        [a -> c -> b -> d]
        '''
        if idx > len(self) or idx < 0:
            raise IndexError('Index out of bounds: {0}'.format(idx))
        prev, current = self._nodes_at(idx)
        self._insert_at(value, prev, current)

    def remove(self, idx):
        '''
        Remove an index from the list

        >>> s = SinglyLinkedList()
        >>> s.insert('a', 0)
        >>> s.remove(0)
        >>> s
        []
        >>> len(s)
        0
        >>> s.insert('b', 0)
        >>> s.insert('c', 1)
        >>> s.insert('d', 2)
        >>> s.remove(1)
        >>> s
        [b -> d]
        >>> s.remove(2)
        Traceback (most recent call last):
        ...
        IndexError: Index out of bounds: 2
        '''
        if idx < 0 or idx >= self._size:
            raise IndexError('Index out of bounds: {0}'.format(idx))
        prev, current = self._nodes_at(idx)
        self._remove_node(prev, current)

    def prepend(self, value):
        '''
        Add an element to the beginning (cheap)

        >>> s = SinglyLinkedList(range(3))
        >>> s.prepend(5)
        >>> s
        [5 -> 0 -> 1 -> 2]
        '''
        self.insert(value, 0)

    def append(self, value):
        '''
        Add an element to the end (expensive)

        >>> s = SinglyLinkedList(range(3))
        >>> s.append(5)
        >>> s
        [0 -> 1 -> 2 -> 5]
        '''
        self.insert(value, self._size)

    def extend(self, iterable):
        '''
        Add all elements of iterable to the linked list

        >>> s = SinglyLinkedList()
        >>> s.extend([1, 2, 3, 4, 5])
        >>> s
        [1 -> 2 -> 3 -> 4 -> 5]
        >>> s.extend(['a', 'b', 'c'])
        >>> s
        [1 -> 2 -> 3 -> 4 -> 5 -> a -> b -> c]
        >>> len(s)
        8
        '''
        prev, _ = self._nodes_at(self._size)
        for value in iterable:
            new_node = self._insert_at(value, prev, None)
            prev = new_node

    def push(self, value):
        self.prepend(value)

    def pop(self):
        if self._size == 0:
            raise IndexError('Cannot pop an empty list')
        value = self._head._value
        self._head = self._head._next
        return value

    def __str__(self):
        '''
        Convert this to a string
        This is called when you do "str(linkedlist)"

        >>> s = SinglyLinkedList()
        >>> print(s)
        []
        >>> s.insert('a', 0)
        >>> s.insert('b', 0)
        >>> s.insert('c', 0)
        >>> print(s)
        [c -> b -> a]
        '''
        return '[' + ' -> '.join([str(x) for x in self]) + ']'

    def __repr__(self):
        return str(self)

    class _Iterator(object):
        'Iterator over the linked list'

        def __init__(self, linkedlist):
            'Initialize to the beginning'
            self.current = linkedlist._head

        def __next__(self):
            'Return the next element'
            if self.current is None:
                raise StopIteration
            current_value = self.current._value
            self.current = self.current._next
            return current_value

    def __iter__(self):
        '''
        return an iterator (must implement __next__)

        >>> s = SinglyLinkedList('abc')
        >>> for x in s:
        ...     print(x)
        ...
        a
        b
        c
        '''
        return self._Iterator(self)

    def __add__(a, b):
        '''
        Return a new linked-list with contents from a then contents from b.
        This is called when you do "a + b".

        >>> a = SinglyLinkedList([1, 2, 3])
        >>> b = SinglyLinkedList(['a', 'b', 'c'])
        >>> a + b
        [1 -> 2 -> 3 -> a -> b -> c]
        >>> a
        [1 -> 2 -> 3]
        >>> b
        [a -> b -> c]
        >>> len(a + b)
        6
        '''
        newlist = SinglyLinkedList(a)
        newlist.extend(b)
        return newlist

    def __contains__(self, value):
        '''
        Return true if value is in this container.  False otherwise.
        This is called if you say "value in self"

        >>> 2 in SinglyLinkedList(range(10))
        True
        >>> 20 in SinglyLinkedList(range(10))
        False
        '''
        return any(x == value for x in self)

    def __delitem__(self, idx):
        '''
        Remove the item at index idx.
        This is called if you say "del linkedlist[i]" (I think)

        Note: if called like so: "del linkedlist[i:j]", then idx is a slice
        object with attributes "start", "stop", and "step".

        >>> s = SinglyLinkedList([1,2,3])
        >>> del s[1]
        >>> s
        [1 -> 3]
        '''
        self.remove(idx)

    def _slice_indices(self, idxs):
        start = idxs.start
        stop = idxs.stop
        step = idxs.step
        start = 0 if idxs.start is None else idxs.start
        stop = len(self) if idxs.stop is None else min(len(self), idxs.stop)
        if start < 0:
            start = len(self) - start
        if stop < 0:
            stop = len(self) + 1 + stop
        step = (1 if start <= stop else -1) if idxs.step is None else idxs.step
        if step < 0:
            if idxs.start is None:
                start = len(self) - 1
            if idxs.stop is None:
                stop = -1
        return (range(start, stop, step), step)

    def __getitem__(self, idx):
        '''
        Return item at index idx.
        This is called if you say "linkedlist[i]"

        Note: if called like so: "linkedlist[i:j]", then idx is a slice object
        with attributes "start", "stop", and "step".

        >>> s = SinglyLinkedList(range(10))
        >>> s[1:5]
        [1 -> 2 -> 3 -> 4]
        >>> s[::-1]
        [9 -> 8 -> 7 -> 6 -> 5 -> 4 -> 3 -> 2 -> 1 -> 0]
        >>> s[5::-1]
        [5 -> 4 -> 3 -> 2 -> 1 -> 0]
        >>> s[:5:-1]
        [9 -> 8 -> 7 -> 6]
        '''
        if not isinstance(idx, slice):
            _, node = self._nodes_at(idx)
            return node._value
        idx_range, step = self._slice_indices(idx)
        returned = SinglyLinkedList()
        current = self._head
        location = 0
        if step < 0:
            indices = sorted(idx_range)
            for i in indices:
                while location < i:
                    current = current._next
                    if location >= len(self):
                        return returned # return early if we are outside
                    location += 1
                returned.prepend(current._value)
        else:
            ret_prev = None
            for i in idx_range:
                while location < i:
                    if location >= len(self):
                        return returned # return early if we are outside
                    current = current._next
                    location += 1
                ret_prev = returned._insert_at(current._value, ret_prev, None)
        return returned

    def __setitem__(self, idx, value):
        '''
        Replace the value at index idx to value.
        This is called if you say "linkedlist[idx] = value"

        Note: if called like so: "linkedlist[i:j] = sequence", then idx is a
        slice object with attributes "start", "stop", and "step".

        >>> s = SinglyLinkedList(range(5))
        >>> s[1] = 'a'
        >>> s
        [0 -> a -> 2 -> 3 -> 4]
        >>> s[::-1] = range(1, 6)
        >>> s
        [5 -> 4 -> 3 -> 2 -> 1]
        >>> s[::2] = [1]
        Traceback (most recent call last):
        ...
        ValueError: attempt to assign sequence of size 1 to extended slice of size 3
        '''
        if not isinstance(idx, slice):
            _, node = self._nodes_at(idx)
            node._value = value
            return
        idx_range, step = self._slice_indices(idx)
        #idx_range = SinglyLinkedList(idx_range)
        nodes_to_replace = SinglyLinkedList()
        location = 0
        current = self._head
        if step < 0:
            indices = sorted(idx_range)
            for i in indices:
                while location < i:
                    current = current._next
                    if location >= len(self):
                        break
                    location += 1
                if location >= len(self):
                    break
                nodes_to_replace.prepend(current)
        else:
            ret_prev = None
            for i in idx_range:
                while location < i:
                    if location >= len(self):
                        break
                    current = current._next
                    location += 1
                if location >= len(self):
                    break
                ret_prev = nodes_to_replace._insert_at(current, ret_prev, None)
        if len(nodes_to_replace) != len(value):
            raise ValueError('attempt to assign sequence of size {0} to extended'
                             ' slice of size {1}'.format(
                                 len(value), len(nodes_to_replace)))
        for node, val in zip(nodes_to_replace, value):
            node._value = val

    def __eq__(self, other):
        '''
        Return True if the other linked list is the same as this one.
        This is called if you say "a == b"
        >>> a = SinglyLinkedList([1, 2, 3])
        >>> b = SinglyLinkedList(range(1, 4))
        >>> a == b
        True
        >>> b.insert('a', 2)
        >>> a == b
        False
        '''
        return all(a == b for a, b in zip(self, other))

    def __ne__(self, other):
        '''
        Return True if the other linked list is the same as this one.
        This is called if you say "a != b"
        >>> a = SinglyLinkedList([1, 2, 3])
        >>> b = SinglyLinkedList(range(1, 4))
        >>> a != b
        False
        >>> b.insert('a', 2)
        >>> a != b
        True
        '''
        return any(a != b for a, b in zip(self, other))

