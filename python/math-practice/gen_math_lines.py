#!/usr/bin/env python3
'''
Generates a note card CSV file for specific math problems
'''

import argparse
import csv
import sys

def populate_parser(parser=None):
    'Populate the argument parser'
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
        Generates a note card CSV file with math problems.  You specify the
        type of problem and the bounds on each of the two numbers.
        '''
    parser.formatter_class = argparse.ArgumentDefaultsHelpFormatter

    parser.add_argument('type', choices=('addition', 'multiplication'),
                        help='Type of math problems to generate.')
    parser.add_argument('a', type=int, help='first number lower bound')
    parser.add_argument('b', type=int, help='first number upper bound')
    parser.add_argument('c', type=int, help='second number lower bound')
    parser.add_argument('d', type=int, help='second number upper bound')

    parser.add_argument('-o', '--output', type=argparse.FileType('w'),
                        default=sys.stdout,
                        help='output to this file instead of stdout')

    return parser

def gen_rows(type, a, b, c, d):
    'Generate all rows of question and answer'
    for x in range(a, b+1):
        for y in range(c, d+1):
            if type == 'addition':
                yield {'question': f'{x} + {y} =',
                       'answer': x + y}
                yield {'question': f'{y} + {x} =',
                       'answer': y + x}
            elif type == 'multiplication':
                yield {'question': f'{x} x {y} =',
                       'answer': x * y}
                yield {'question': f'{y} x {x} =',
                       'answer': y * x}

def main(arguments):
    'Main logic here'
    parser = populate_parser()
    args = parser.parse_args(arguments)
    writer = csv.DictWriter(args.output,
                            fieldnames=('question', 'answer'),
                            )#dialect=csv.unix_dialect())
    writer.writeheader()
    writer.writerows(gen_rows(args.type, args.a, args.b, args.c, args.d))
    
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
