#!/usr/bin/env python3
'''
Diff two binary files byte by byte.  Do not try to do insertions or deletions,
just a straight side-by-side comparison.
'''

import sys
import argparse

def parse_args(arguments):
    'Parse and return parsed arguments'
    parser = argparse.ArgumentParser(
        description='''
            Diff two binary files byte-by-byte.  This is a simple comparison
            operation, so no attempts to align based on insertions or
            deletions, just a straight side-by-side comparison.
            ''')
    parser.add_argument('file1')
    parser.add_argument('file2')
    args = parser.parse_args(arguments)
    return args

def count_byte_diffs(file1, file2):
    'Return # bytes different between file1 and file2 side-by-side'
    diff_bytes = 0
    with open(file1, 'rb') as fin1:
        with open(file2, 'rb') as fin2:
            while True:
                c1 = fin1.read(1)
                c2 = fin2.read(1)

                # Handle end of file
                if c1 == bytes():
                    return diff_bytes + len(c2) + len(fin2.read())
                if c2 == bytes():
                    return diff_bytes + len(c1) + len(fin1.read())

                # Diff
                if c1 != c2:
                    diff_bytes += 1

def main(arguments):
    'Main logic here'
    args = parse_args(arguments)
    diff_bytes = count_byte_diffs(args.file1, args.file2)
    print(diff_bytes, 'bytes are different')

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
