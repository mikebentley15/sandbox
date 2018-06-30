#!/usr/bin/env python3

import subprocess as subp
import sys

def tabulate(iterable, width, padding=2):
    '''
    Takes an iterable and tabulates it into columns to fit within the width
    parameter.  Returns a list of strings, one for each line.
    '''
    words = [str(x) for x in iterable]
    shortest_len = min(len(x) for x in words)
    longest_len = max(len(x) for x in words)
    max_columns = (width + padding) // (shortest_len + padding)
    min_columns = (width + padding) // (longest_len + padding)

    # now try all of the columns from max to min.  We want the largest number
    # of columns that fits within the width.
    for cols in range(max_columns, min_columns - 1, -1):
        table = tabulate_cols(words, cols, padding)
        if max(len(x) for x in table) <= width:
            return table
    raise RuntimeError('Something went wrong - could not get it to fit')

def tabulate_cols(iterable, ncols, padding=2):
    '''
    Takes an iterable and tabulates it into the specified number of columns.
    Returns a list of strings, one for each line.

    >>> words = ['a', 'b', 'cccccccc', 'dd', 'e', 'fffffff', 'gg', 'h', 'i', 'j']
    >>> print('\\n'.join(tabulate_cols(words, 3)))
    a         e        i
    b         fffffff  j
    cccccccc  gg
    dd        h

    >>> print('\\n'.join(tabulate_cols([1,2,3], 4)))
    1  2  3
    '''
    words = [str(x) for x in iterable]
    columns = columnate(words, ncols)
    rows = rowate(words, ncols)
    
    widths = [max([0] + [len(y) for y in x]) for x in columns]
    table = []
    pad = ' ' * padding
    for row in rows:
        table.append(pad.join(word.ljust(width)
                              for word, width in zip(row, widths)) 
                        .strip())
    return table

def columnate(elements, ncols):
    '''
    Takes an iterable and returns a list of columns from it.

    >>> words = ['a', 'b', 'cccccccc', 'dd', 'e', 'fffffff', 'gg', 'h', 'i', 'j']
    >>> columnate(words, 3)
    [['a', 'b', 'cccccccc', 'dd'], ['e', 'fffffff', 'gg', 'h'], ['i', 'j']]
    '''
    height = len(elements) // ncols + (1 if len(elements) % ncols > 0 else 0)
    columns = [elements[i:i+height] for i in range(0, len(elements), height)]
    return columns

def rowate(iterable, ncols):
    '''
    Takes an iterable and returns a list of rows that puts it in the columns
    correctly.

    >>> words = ['a', 'b', 'cccccccc', 'dd', 'e', 'fffffff', 'gg', 'h', 'i', 'j']
    >>> rowate(words, 3)
    [['a', 'e', 'i'], ['b', 'fffffff', 'j'], ['cccccccc', 'gg'], ['dd', 'h']]
    '''
    words = list(iterable)
    height = len(words) // ncols + (1 if len(words) % ncols > 0 else 0)
    rows = [words[i::height] for i in range(height)]
    return rows

def terminal_width():
    '''
    Another implementation I could pull and use is jtriley/terminalsize.py
    His implementation is portable between Windows, Mac, and Linux.

      https://gist.github.com/jtriley/1108174

    But my implementation is sufficient for now for my needs.

    Query and return the length of the terminal
    '''
    return int(subp.check_output(['tput', 'cols']).decode('utf-8'))

def main(arguments):
    '''
    Tabulates the standard input into columns for the width of the terminal.
    '''
    width = terminal_width()
    words = [x.strip() for x in sys.stdin.readlines()]
    print('\n'.join(tabulate(words, width)))

if __name__ == '__main__':
    main(sys.argv)
