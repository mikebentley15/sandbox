cdef extern from 'SinglyLinkedList.h':
    struct SinglyLinkedList:
        size_t size
    ctypedef SinglyLinkedList cSLL

    cSLL* SLL_create();
    void  SLL_del(cSLL** listptr);
    void  SLL_insert(cSLL* list, void* value, int idx);
    void  SLL_push(cSLL* list, void* value);
    void* SLL_remove(cSLL* list, int idx);
    void* SLL_pop(cSLL* list);
    void  SLL_clear(cSLL* list);
    cSLL  SLL_iter_at(cSLL* list, int idx);
    void* SLL_at(const cSLL* list, int idx);


cdef class SLL_Iterator:
    cdef SinglyLinkedList _sll

    def __cinit__(self, SLL sll):
        self._sll = SLL_iter_at(sll._sll, 0)

    def __dealloc__(self):
        pass

    def __next__(self):
        cdef void* value
        if self._sll.size == 0:
            raise StopIteration
        value = SLL_at(&self._sll, 0)
        self._sll = SLL_iter_at(&self._sll, 1)
        return <object>value

cdef class SLL:
    '''
    Singly-Linked List implemented in C
    '''

    cdef SinglyLinkedList* _sll

    def __cinit__(self, values=None):
        cdef list value_list = None
        self._sll = SLL_create()
        if values is not None:
            value_list = list(values)
            for x in reversed(value_list):
                self.push(x)

    def __dealloc__(self):
        if self._sll != NULL:
            SLL_del(&self._sll)

    cdef __sanitize_index(self, int idx, bint adding):
        cdef str err_msg = 'linked list index out of range'
        cdef int size = <int>self._sll.size
        if adding:
            if idx > size:
                raise IndexError(err_msg)
            if idx < 0:
                if idx+1 < -size:
                    raise IndexError(err_msg)
                idx = size + idx
            return idx
        else:
            if idx >= size:
                raise IndexError(err_msg)
            if idx < 0:
                if idx < -size:
                    raise IndexError(err_msg)
                idx = size + idx
            return idx

    cpdef insert(self, object value, int idx):
        idx = self.__sanitize_index(idx, True)
        SLL_insert(self._sll, <void*>value, idx)

    cpdef push(self, object value):
        SLL_push(self._sll, <void*>value)

    cpdef pop(self, int idx=0):
        idx = self.__sanitize_index(idx, False)
        return <object>SLL_remove(self._sll, idx)

    cpdef clear(self):
        SLL_clear(self._sll)

    cpdef at(self, int idx):
        print('at', idx)
        idx = self.__sanitize_index(idx, False)
        return <object>SLL_at(self._sll, idx)

    cpdef reverse(self):
        '''
        Reverse *IN PLACE*
        
        >>> ll = SLL()
        >>> ll.reverse()
        >>> ll
        []
        
        >>> ll = SLL([1, 2, 3])
        >>> ll.reverse()
        >>> ll
        [3 -> 2 -> 1]
        '''
        cdef SLL other = SLL()
        for x in self:
            other.push(x)
        self._sll, other._sll = other._sll, self._sll

    def __iter__(self):
        return SLL_Iterator(self)

    def __eq__(self, other):
        return len(self) == len(other) and \
               all(self[i] == other[i] for i in range(len(self)))

    def __hash__(self):
        raise TypeError("unhashable type: 'SLL'")

    def __ne__(self, other):
        return not (self == other)

    def __reversed__(self):
        '''
        Return a reversed version
        
        >>> ll = SLL()
        >>> SLL(reversed(ll))
        []

        >>> ll = SLL([1, 2, 3])
        >>> SLL(reversed(ll))
        [3 -> 2 -> 1]
        >>> ll
        [1 -> 2 -> 3]
        '''
        cdef SLL new_sll = SLL()
        for x in self:
            new_sll.push(x)
        # TODO: figure out why this segfaults when I just return new_sll
        # TODO- >>> [x for x in reversed(SLL([1,2,3]))]
        for x in new_sll:
            yield x

    def __add__(self, other):
        cdef SLL new_list = SLL()
        for x in reversed(self):
            new_list.push(x)
        for x in reversed(other):
            new_list.push(x)
        return new_list

    def __mul__(self, int how_many):
        cdef:
            int i
            SLL reversed_copy = None
            SLL return_value = None

        if how_many <= 0:
            return SLL()
        if how_many == 1:
            # TODO: return self.copy()
            return SLL(self)

        reversed_copy = SLL(reversed(self))
        return_value = SLL()
        for i in range(how_many):
            for x in reversed_copy:
                return_value.push(x)

        return return_value

    def __rmul__(self, int how_many):
        return self.__mul__(how_many)

    cdef _slice_indices(self, slice idxs):
        start = 0 if idxs.start is None else idxs.start
        stop = len(self) if idxs.stop is None else min(len(self), idxs.stop)
        step = (1 if start <= stop else -1) if idxs.step is None else idxs.step
        if start < 0:
            start = len(self) - start
        if stop < 0:
            stop = len(self) + 1 + stop
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

        >>> s = SLL(range(10))
        >>> s[1:5]
        [1 -> 2 -> 3 -> 4]
        >>> s[::-1]
        [9 -> 8 -> 7 -> 6 -> 5 -> 4 -> 3 -> 2 -> 1 -> 0]
        >>> s[5::-1]
        [5 -> 4 -> 3 -> 2 -> 1 -> 0]
        >>> s[:5:-1]
        [9 -> 8 -> 7 -> 6]
        '''
        cdef:
            SLL returned = None
            int step, location
            list indices = None
            SLL_Iterator iterator = None
            object value = None

        if not isinstance(idx, slice):
            return self.at(idx)

        idx_range, step = self._slice_indices(idx)
        returned = SLL()

        if step < 0:
            indices = sorted(idx_range)
        else:
            indices = list(idx_range)

        iterator = SLL_Iterator(self)
        location = -1
        for i in indices:
            while location < i:
                value = next(iterator)
                if location >= len(self):
                    return returned # return early if outside
                location += 1
            returned.push(value)

        if step >= 0:
            returned = SLL(reversed(returned))

        return returned

    def __setitem__(self, int idx, object value):
        # TODO: handle idx being a slice and value being an iterator
        self.pop(idx)
        self.insert(value, idx)

    def __contains__(self, object value):
        return any(x == value for x in self)

    def __len__(self):
        return self._sll.size

    def __str__(self):
        '''
        Convert the linked list to a string

        >>> str(SLL())
        '[]'
        >>> str(SLL([1, 2, 3]))
        '[1 -> 2 -> 3]'
        >>> SLL(('a', 'b', 'c', 'd'))
        ['a' -> 'b' -> 'c' -> 'd']
        '''
        return '[' + ' -> '.join(str(x) for x in self) + ']'

    def __repr__(self):
        '''
        Convert the linked list to a string

        >>> repr(SLL())
        '[]'
        >>> repr(SLL([1, 2, 3]))
        '[1 -> 2 -> 3]'
        >>> SLL(('a', 'b', 'c', 'd'))
        ['a' -> 'b' -> 'c' -> 'd']
        '''
        return '[' + ' -> '.join(repr(x) for x in self) + ']'

    def __delitem__(self, int idx):
        self.pop(idx)

