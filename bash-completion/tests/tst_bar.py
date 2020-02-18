#!/usr/bin/env python3

import abc
import argparse
import collections
import os
import sys
import unittest

import arginspect
from completion import get_completion
from pprintable import PPrintable
import testutil as util

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
with open(os.path.join(SCRIPT_DIR, '..', 'bin', 'bar'), 'r') as fin:
    bar = util.load_module_from_string('bar', fin.read())
del fin


class TestArgparse_Bar(arginspect.ArgParseTestBase):
    BAR_PROG = 'bar'
    BAR_BASH_COMPLETION = os.path.join(SCRIPT_DIR, '..', 'completions', 'bar')

    def bashcomplete(self, args):
        return get_completion(self.BAR_BASH_COMPLETION, self.BAR_PROG, args)

    def test_empty_available_options(self):
        self.assertEmptyAvailableOptions(bar.populate_parser())

    def test_empty_available_options_for_subparsers(self):
        self.assertEachSubparserEmptyAvailableOptions(bar.populate_parser())

    def test_no_positional_args(self):
        inspector = arginspect.ParserInspector(bar.populate_parser())
        # test that there are no positional arguments
        self.assertEqual(inspector.position_actions, [])

if __name__ == '__main__':
    unittest.main()
