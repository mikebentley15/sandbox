'''
The definition of a SinglyLinkedList class
'''

class SinglyLinkedList(object):
    '''
    A singly-linked-list meaning we only have links forward and not backward.
    '''

    def __init__(self, iterable=None):
        '''
        Initialize the list, optionally with initial elements
        '''
        pass

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
        pass

    def __repr__(self):
        '''
        Convert this to a string
        This is called when you do "repr(linkedlist)"

        >>> s = SinglyLinkedList()
        >>> print(repr(s))
        []
        >>> s.insert('a', 0)
        >>> s.insert('b', 0)
        >>> s.insert('c', 0)
        >>> print(repr(s))
        [c -> b -> a]
        '''
        pass

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
        pass

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
        pass

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
        pass

    def prepend(self, value):
        '''
        Add an element to the beginning (cheap)

        >>> s = SinglyLinkedList(range(3))
        >>> s.prepend(5)
        >>> s
        [5 -> 0 -> 1 -> 2]
        '''
        pass

    def append(self, value):
        '''
        Add an element to the end (expensive)

        >>> s = SinglyLinkedList(range(3))
        >>> s.append(5)
        >>> s
        [0 -> 1 -> 2 -> 5]
        '''
        pass

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
        pass

    def push(self, value):
        pass

    def pop(self):
        pass

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
        pass

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
        pass

    def __contains__(self, value):
        '''
        Return true if value is in this container.  False otherwise.
        This is called if you say "value in self"

        >>> 2 in SinglyLinkedList(range(10))
        True
        >>> 20 in SinglyLinkedList(range(10))
        False
        '''
        pass

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
        pass

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
        pass

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
        pass

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
        pass

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
        pass

