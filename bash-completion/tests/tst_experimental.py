#!/usr/bin/env python3

import argparse
import collections
import os
import sys
import unittest

from arginspect import ParserInspector
from completion import get_completion
from pprintable import PPrintable
import testutil as util

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
with open(os.path.join(SCRIPT_DIR, '..', 'bin', 'bar'), 'r') as fin:
    bar = util.load_module_from_string('bar', fin.read())
del fin

class TestArgparseIntrospection(unittest.TestCase):

    def populate_args(self, parser):
        'Populate a parser'
        # Store each type of action
        # - _StoreAction       'store'
        # - _StoreConstAction  'store_const'
        # - _StoreTrueAction   'store_true'
        # - _StoreFalseAction  'store_false'
        # - _AppendAction      'append'
        # - _AppendConstAction 'append_const'
        # - _CountAction       'count'
        # - _HelpAction        'help'
        # - _VersionAction     'version'
        # - _SubParsersAction  'parsers'
        # - _ExtendAction      'extend'
        parser.add_argument('-k', '--biggest', action='store')
        parser.add_argument('-j3', dest='jobs', action='store_const', const=3)
        parser.add_argument('--no-debug', dest='debug', action='store_false')
        parser.add_argument('-d', '--debug', dest='debug', action='store_true')
        parser.add_argument('-x', dest='exes', action='append')
        parser.add_argument('--one', const=1, action='append_const')
        parser.add_argument('-v', '--verbose', action='count')
        parser.add_argument('--version', version='1.2.3', action='version')
        parser.add_argument('--foo', nargs='+', action='extend')
        parser.add_argument('positional')
        return parser

    def test_option_strings(self):
        parser = argparse.ArgumentParser()
        self.populate_args(parser)
        intro = ParserInspector(parser)
        self.assertEqual(set(intro.option_strings), set([
            '-h', '--help',
            '-k', '--biggest',
            '-j3',
            '--no-debug',
            '-d', '--debug',
            '-x',
            '--one',
            '-v', '--verbose',
            '--version',
            '--foo',
            ]))

    def test_option_strings_subparsers(self):
        parser = self.populate_args(argparse.ArgumentParser())
        sub = parser.add_subparsers()
        p_a = self.populate_args(sub.add_parser('--a'))
        p_b = self.populate_args(sub.add_parser('b'))
        intro = ParserInspector(parser)
        expected_option_strings = [
            '-h', '--help',
            '-k', '--biggest',
            '-j3',
            '--no-debug',
            '-d', '--debug',
            '-x',
            '--one',
            '-v', '--verbose',
            '--version',
            '--foo',
            ]
        self.assertEqual(set(ParserInspector(parser).option_strings),
                         set(expected_option_strings))
        self.assertEqual(set(ParserInspector(p_a).option_strings),
                         set(expected_option_strings))
        self.assertEqual(set(ParserInspector(p_b).option_strings),
                         set(expected_option_strings))

if __name__ == '__main__':
    unittest.main()
