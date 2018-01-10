def HashTable() :
    '''
    Hash table is a container that uses the __hash__() function of objects
    passed in.  Its internal storage is a Python list that remains a static
    size until this class decides it want to change the capacity (for reduction
    in collisions).

    Collisions are dealt with using linked lists.

    Note, there is no enforced ordering of elements in this container and is
    able to change orderings of elements at any time.
    '''

    def __init__(self, capacity=20):
        '''
        Initializes the hash table to be able to comfortably contain at least
        capacity number of elements with constant-time insertion and removal.
        '''
        pass

    def __len__(self):
        'return number of elements'
        pass

    def __str__(self):
        'return str(self)'
        pass

    def __repr__(self):
        'return repr(self)'
        pass

    def __contains__(self, value):
        pass

    def add(self, value):
        '''
        add the value to the hash table if it is not already there, otherwise
        do nothing
        '''
        pass

    def update(self, iterable):
        '''
        add all elements from iterable into the set, if they are not already
        there
        '''
        pass

    def discard(self, value):
        'remove value from the hash table if it is there, otherwise do nothing'
        pass

    def remove(self, value):
        'remove value from the hash table.  If it is not there, raise KeyError'
        pass

    def clear(self):
        'remove all elements'
        pass

    def pop(self):
        '''
        remove and return an arbitrary element.  Raises KeyError if it is
        empty
        '''
        pass

    def copy(self):
        'return a shallow copy'
        pass

    def __eq__(self, other_table):
        'return self == other_table'
        pass

    def __ne__(self, other_table):
        'return self != other_table'
        pass

    class _Iterator():
        'Iterator for HashTable'

        def __init__(self, hash_table):
            'Initialize to the beginning of the hash table'
            pass

        def __next__(self):
            'Return the next value and update the iterator pointer'
            pass

    def __iter__(self):
        'return iterator.  This allows this container to be iterable'
        return _Iterator(self)

