#!/usr/bin/env python3

import argparse
import sys
import itertools

def populate_parser(parser=None):
    if not parser:
        parser = argparse.ArgumentParser()
    parser.description = 'Calculate multiplicative permanence'
    group = parser.add_mutually_exclusive_group()
    group.add_argument('num', type=int, default=277777788888899, nargs='?',
                        help='''
                            Show the calculations of multiplicative permanence
                            of this number.  The default is the smallest one of
                            permanence 11 (i.e., 277,777,788,888,899).
                            ''')
    group.add_argument('-s', '--search', action='store_true',
                        help='''
                            Search infinitely instead of the one number.  Press
                            CTRL-C when you want to exit.  Otherwise, it will
                            run forever.''')
    parser.add_argument('--print-bigger', type=int, default=3,
                        help='''
                            For --search.  Only print numbers with depth higher
                            than this.
                            ''')
    parser.add_argument('--start-length', type=int, default=1,
                        help='''
                            For --search, start with this many factors instead
                            of starting from the beginning.
                            ''')
    group.add_argument('--reverse-search', type=int,
                        help='''
                            Search infinitely looking for numbers that generate
                            a number with the same digit factors as the given
                            number here.  For example, if 8 is given, it will
                            look for numbers whose digits produce 8, 24, 222,
                            42, 18, 81, 124, ...  This is useful for looking
                            for the next higher multiplicative permanence from
                            a high one you've found.
                            ''')
    return parser

# TODO: is it faster to do collections.Counter instead of list?
# TODO- product would be prod *= factor**power for factor, power in digits.items()
def to_digits(num):
    'Convert a number to a list of digits'
    if num <= 0:
        return [0]
    digits = []
    while num > 0:
        digits.append(num % 10)
        num = num // 10
    digits = list(reversed(digits))
    return digits

def digits_to_string(digits):
    return ''.join(str(d) for d in digits)

def product(array):
    prod = 1
    for x in array:
        prod *= x
    return prod

def per(digits, quiet=True):
    '''
    Calculates the depth of multiplicative permanence until a single digit.

    @param digits (list(int)): list of digits of the current number, each
        between 0 and 9.
    @param quiet (bool): False means to print the numbers as they are found

    @return (int) multiplicative permanence.
    '''
    depth = 0
    if not quiet:
        print('00: {}'.format(digits_to_string(digits)))
    while len(digits) > 1:
        depth += 1
        num = product(digits)
        if not quiet:
            print(f'{depth:02}: {num}')
        digits = to_digits(num)
    return depth

def generate_draws(choices, start_len=1):
    '''
    An infinite generator of lists of draws with replacement starting from one
    draw and methodically working up to more and more samples.
    '''
    for count in itertools.count(start=start_len):
        yield from itertools.combinations_with_replacement(choices, count)

def prime_digits(digits):
    assert all(0 < x < 10 for x in digits)
    pdigits = []
    for x in digits:
        if x < 2:
            pass
        elif x == 4: pdigits.extend([2, 2])
        elif x == 6: pdigits.extend([2, 3])
        elif x == 8: pdigits.extend([2, 2, 2])
        elif x == 9: pdigits.extend([3, 3])
        else:        pdigits.append(x)
    pdigits.sort()
    return pdigits

def prime_digits_to_string(pdigits):
    s = ''
    for factor in (2, 3, 5, 7):
        if factor in pdigits:
            if s:
                s += ' '
            s += '{}^{}'.format(factor, pdigits.count(factor))
    return s

def digit_combos(pdigits):
    'Create all unique unordered combos equivalent to the given prime digits'
    pdigits = prime_digits(pdigits) # just to be sure they're prime
    combos = set([tuple(pdigits)])

    # combine twos to make fours
    new_combos = list(combos)
    for combo in combos:
        num_twos = combo.count(2)
        remaining = [x for x in combo if x != 2]
        

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    if args.search:
        best_match = (0, 0, 0)
        last_tested = 0
        current_len = 0
        try:
            #for factors in generate_draws([2, 3, 5, 7]):
            for factors in generate_draws([2, 3, 7], start_len=args.start_length):
                if len(factors) > current_len:
                    current_len = len(factors)
                    print('starting length', current_len)
                last_tested = factors
                depth = per(factors, quiet=True)
                if depth > args.print_bigger:
                    current = prime_digits_to_string(factors)
                    print(f'  {current}: {depth}')
                if depth > best_match[2]:
                    best_match = (len(factors), prime_digits_to_string(factors), depth)
        except KeyboardInterrupt:
            last_tested_str = prime_digits_to_string(last_tested)
            print()
            print()
            print(f'best match:  ({best_match[0]}) {best_match[1]} '
                  f'@ {best_match[2]}')
            print(f'last tested: ({len(last_tested)}) {last_tested_str} '
                  f'@ {per(last_tested, quiet=True)}')
            print()
    elif args.reverse_search:
        print(f'reverse search of {args.reverse_search}')
        digits = to_digits(args.reverse_search)
        if digits.count('0'):
            print(f'  Error: {args.reverse_search} has a zero in it, exiting')
        pdigits = prime_digits(digits)
        print(f'  prime digits:            {prime_digits_to_string(pdigits)}')
        pdigit_combos = prime_digit_combos(pdigits)
        print(f'  # 1-digit factor combos: {len(pdigit_combos)}')
        #print('reverse search not yet implemented')
        #print(args)
        #return 1
    else:
        per(to_digits(args.num), quiet=False)

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
