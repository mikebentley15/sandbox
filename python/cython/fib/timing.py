from fib import python_fib, cpython_fib, c_fib

def main():
    from timeit import timeit
    print('python_fib(90):  {}'.format(timeit(lambda: python_fib(90))))
    print('cpython_fib(90): {}'.format(timeit(lambda: cpython_fib(90))))
    print('c_fib(90):       {}'.format(timeit(lambda: c_fib(90))))

if __name__ == '__main__':
    main()

