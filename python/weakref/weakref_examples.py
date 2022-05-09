import weakref

import pydoc
import gc

class A:
    'Wrapper that prints creation and deletion'
    def __init__(self, data):
        self.data = data
        print(f'  A created: {self}')
    def __del__(self): print(f'  A deleted: {self}')
    def __str__(self): return f'A@{hex(id(self))}({self.data})'


def ex01():
    'example from https://www.geeksforgeeks.org/weak-references-in-python/'
    # creating object of a class
    class ListObj(list): pass
    obj = ListObj([1, 2, 3])
 
    # creating a normal list object
    normal_list = obj
    print(f"  This is a normal object: {normal_list}")
     
    # this returns a weak reference to obj
    weak_list = weakref.ref(obj)
    weak_list_obj = weak_list()
    print(f"  This is a object created using weak reference: {weak_list_obj}")
     
    # creating a proxy of original object
    proxy_list = weakref.proxy(obj)
    print(f"  This is a proxy object: {proxy_list}")
     
    # printing the count of weak references
    for objects in [normal_list, weak_list_obj, proxy_list]:
        print(f"  Number of weak references: {weakref.getweakrefcount(objects)}")

def ex02():
    'proxy object after deletion'
    a = A(42)
    print('  creating proxy')
    p = weakref.proxy(a)
    print(f'    a: {a}')
    print(f'    p: {p}')
    print(f'  deleting a')
    del a
    gc.collect()
    try:
        print(f'    p: {p}') # should not work
    except ReferenceError as ex:
        print(f'    could not deref p: {ex}')
    print('  done')

def weakref_if_else(w, if_func, else_func):
    w_val = w()
    if w_val is not None:
        if_func(w_val)
    else:
        else_func(w_val)

def print_weakref(w, name='w'):
    weakref_if_else(w,
        if_func=lambda x: print(f'    {name}: {x}'),
        else_func=lambda x: print(f'    {name} is no longer a valid weakref'),
    )

def ex03():
    'direct use of weak reference'
    a = A(42)
    print('  creating weak reference')
    w = weakref.ref(a)
    print(f'    a: {a}')
    print_weakref(w)
    print(f'  deleting a')
    del a
    gc.collect()
    print_weakref(w)

def ex04():
    'use of weakref.finalize'
    print('  TODO')

def ex05():
    'use of weakref.WeakKeyDictionary'
    print('  TODO')

def ex06():
    'use of weakref.WeakValueDictionary'
    print('  TODO')

def main():
    examples = ((k, v) for k, v in globals().items() if k.startswith('ex'))
    for name, func in examples:
        print(f'{name}:')
        print(pydoc.getdoc(func))
        func()
        print()

if __name__ == '__main__':
    main()
