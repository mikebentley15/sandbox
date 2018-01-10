'''
>>> from cdatastruct import SinglyLinkedList as SLL

Test the empty constructor
>>> SLL().size
0

Test at() on an empty linked list with a non-integer
>>> SLL().at('mike')
Traceback (most recent call last):
...
TypeError: an integer is required (got type str)

Test at() on an empty linked list out of bounds
>>> SLL().at(0)
Traceback (most recent call last):
...
IndexError: linked list index out of range

Test at() again on an empty linked list out of bounds
>>> SLL().at(-1)
Traceback (most recent call last):
...
IndexError: linked list index out of range

Test converting an empty linked list to a string
>>> str(SLL())
'[]'

Test insert and size for valid cases
>>> ll = SLL()
>>> ll.insert(0, 'a')
>>> ll.size
1

Test insert and str() for a few elements
>>> ll = SLL()
>>> ll.insert(0, 'a')
>>> str(ll)
'[a]'
>>> ll.insert(1, 'b')
>>> str(ll)
'[a -> b]'
>>> ll.insert(1, 'c')
>>> str(ll)
'[a -> c -> b]'
>>> ll.insert(0, 'd')
>>> str(ll)
'[d -> a -> c -> b]'

Test negative insertions
>>> ll.insert(-1, 'e')
>>> str(ll)
'[d -> a -> c -> b -> e]'
>>> ll.insert(-3, 'f')
>>> str(ll)
'[d -> a -> c -> f -> b -> e]'

Test ref count for insert()
>>> t = ('a', 'b', 'c')
>>> ll2 = SLL()
>>> ll2.insert(0, t)
>>> del t
>>> t2 = ll2.remove(0)
>>> t2
('a', 'b', 'c')

Test negative at() indices
>>> ll.at(-1)
'e'
>>> ll.at(-3)
'f'

Test push()
>>> ll.push('g')
>>> str(ll)
'[g -> d -> a -> c -> f -> b -> e]'
>>> del ll
>>> ll2 = SLL()
>>> ll2.push('a')
>>> str(ll2)
'[a]'

Test remove()
>>> SLL().remove(0)
Traceback (most recent call last):
...
IndexError: linked list index out of range
>>> SLL().remove(-1)
Traceback (most recent call last):
...
IndexError: linked list index out of range
>>> ll2.remove(1)
Traceback (most recent call last):
...
IndexError: linked list index out of range
>>> str(ll2)
'[a]'
>>> ll2.remove(0)
'a'
>>> str(ll2)
'[]'
>>> ll2.insert(0, 'a')
>>> ll2.insert(1, 2)
>>> ll2.insert(2, object())
>>> type(ll2.remove(-1))
<class 'object'>
>>> ll2.remove(-2)
'a'
>>> str(ll2)
'[2]'
>>> del ll2

Test pop()
>>> ll = SLL()
>>> ll.push(1)
>>> ll.push(2)
>>> ll.pop()
2
>>> ll.push(3)
>>> ll.push(4)
>>> ll.pop()
4
>>> ll.pop()
3
>>> ll.pop()
1
>>> ll.pop()
Traceback (most recent call last):
...
IndexError: pop from empty linked list

Test clear()
>>> ll = SLL()
>>> ll.clear()
>>> str(ll)
'[]'
>>> ll.push(5)
>>> ll.push(6)
>>> ll.clear()
>>> ll.size
0
>>> str(ll)
'[]'

Test extend()
>>> ll = SLL()
>>> ll.extend(range(5))
>>> str(ll)
'[0 -> 1 -> 2 -> 3 -> 4]'

Test constructor with extend-like behavior
>>> SLL(range(6))
'[0 -> 1 -> 2 -> 3 -> 4 -> 5]'

'''

if __name__ == '__main__':
    print()
    import doctest
    doctest.testmod()
    print('All tests done!')
