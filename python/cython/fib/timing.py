from fib import python_fib, cpython_fib, c_fib, cpp_fib, cfib, cppfib

def local_fib(n):
    a, b = 0.0, 1.0
    for i in range(n):
        a, b = a + b, a
    return a

def main():
    from timeit import timeit
    print('local_fib(90):   {}'.format(timeit(lambda: python_fib(90))))
    print('python_fib(90):  {}'.format(timeit(lambda: python_fib(90))))
    print('cpython_fib(90): {}'.format(timeit(lambda: cpython_fib(90))))
    print('c_fib(90):       {}'.format(timeit(lambda: c_fib(90))))
    print('cfib(90):        {}'.format(timeit(lambda: cfib(90))))

if __name__ == '__main__':
    main()

