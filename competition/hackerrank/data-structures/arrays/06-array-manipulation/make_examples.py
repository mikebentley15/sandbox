#!/usr/bin/env python3

def mk_function_example(fname, n, m, func):
    '''
    Create an example file using a function that takes the row number

    @param fname - filename
    @param n - the bound on entries in lines and is not checked
    @param m - number of lines after the header
    @param func - function taking integer index from 0 to m-1
    '''
    with open(fname, 'w') as fout:
        print(n, m, file=fout)
        for i in range(m):
            print(func(i), file=fout)

def mk_pattern_example(fname, n, m, lines, pattern):
    '''
    Create the example file.
    
    @param fname - filename
    @param n - the bound on entries in lines and is not checked
    @param m - number of lines after the header
    @param lines - lines to repeat
    @param pattern - indices to lines of pattern to repeat
    '''
    pattern_func = lambda i: lines[pattern[i % len(pattern)]]
    return mk_function_example(fname, n, m, pattern_func)

def mk_ex01():
    with open('ex01.txt', 'w') as fout:
        print('5 3\n'
              '1 2 100\n'
              '2 5 100\n'
              '3 4 100',
              file=fout)

def mk_ex02():
    lines = ('1 2 100', '2 5 100', '3 4 100')
    pattern = (0, 0, 1, 2, 0, 1, 2, 0, 1, 2)
    return mk_pattern_example('ex02.txt', 10000000, 200000, lines, pattern)

def mk_ex03():
    lines = ('1 10000000 1000000000',)
    pattern = (0,)
    return mk_pattern_example('ex03.txt', 10000000, 200000, lines, pattern)

def mk_ex04():
    n = 10000000
    m = 200000
    gen_line = lambda idx: '{0} 10000000 1000000000'.format(idx + 1)
    return mk_function_example('ex04.txt', n, m, gen_line)

def mk_ex05():
    n = 10000000
    m = 200000
    gen_line = lambda idx: '1 {0} 1000000000'.format(n - idx)
    return mk_function_example('ex05.txt', n, m, gen_line)

def mk_ex06():
    n = 10000000
    m = 200000
    gen_line = lambda idx: '{0} {1} 1000000000'.format(idx + 1, n - idx)
    return mk_function_example('ex06.txt', n, m, gen_line)

def mk_ex07():
    n = 10000000
    m = 200000
    gen_line = lambda idx: '{0} {1} 1000000000'.format(m - idx, n - idx)
    return mk_function_example('ex07.txt', n, m, gen_line)

def mk_ex08():
    n = 10000000
    m = 200000
    gen_line = lambda idx: '{0} {1} 1000000000'.format(m - idx, n - idx)
    def gen_line(idx):
        format_str = '{0} {1} 1000000000'
        if idx == 0:
            return format_str.format(n, n)
        return format_str.format(m - idx, n - idx)
    return mk_function_example('ex08.txt', n, m, gen_line)

def main():
    mk_ex01()
    mk_ex02()
    mk_ex03()
    mk_ex04()
    mk_ex05()
    mk_ex06()
    mk_ex07()
    mk_ex08()

if __name__ == '__main__':
    main()
