#!/usr/bin/env python3

import argparse

import tst_experimental

def main():
    test = tst_experimental.TestArgparseIntrospection()
    p = test.populate_args(argparse.ArgumentParser())
    sub = p.add_subparsers()
    test.populate_args(sub.add_parser('a'))
    test.populate_args(sub.add_parser('b'))
    i = tst_experimental.ParserInspector(p)
    print(i)

if __name__ == '__main__':
    main()
