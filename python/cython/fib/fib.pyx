import cython

def python_fib(n):
    a, b = 0.0, 1.0
    for i in range(n):
        a, b = a + b, a
    return a

def cpython_fib(int n):
    cdef int i
    cdef double a = 0.0
    cdef double b = 1.0
    for i in range(n):
        a, b = a + b, a
    return a

cdef extern from "cfib.h":
    double cfib(int n)

cdef c_fib(int n):
    return cfib(n)

cdef extern from "cppfib.hpp":
    double cppfib(int n)

cdef cpp_fib(int n):
    return cppfib(n)
