#!/usr/bin/env python3

import argparse
import collections
import os
import pprint
import sys
import unittest

from completion import get_completion
import testutil as util

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))

sys.path.append(os.path.join(SCRIPT_DIR, '..', 'bin'))
import bar
sys.path.pop()

def is_option_action(action):
    return bool(action.option_strings)

def is_position_action(action):
    return not is_option_arg(action)

def is_subparser_action(action):
    return isinstance(action, argparse._SubParsersAction)

class PPrintable:
    def pp(self):
        pprint.pprint(self)

    def __str__(self):
        return pprint.pformat(self)

    def __repr__(self):
        return type(self).__name__ + '(' + \
            ', '.join(name + ': ' + repr(val) for name, val in self._get_kwargs()) + \
            ')'

    def _get_kwargs(self):
        return sorted(self.__dict__.items())

def _pprint_pprintable(printer, obj, stream, indent, allowance, context, level):
    args = obj._get_kwargs()
    write = stream.write
    write(obj.__class__.__name__)
    write('{')
    if args:
        write('\n' + (indent + printer._indent_per_level) * ' ')
        printer._format_dict_items(args, stream, indent, allowance+1, context,
                                   level)
    write('}')

pprint.PrettyPrinter._dispatch[PPrintable.__repr__] = _pprint_pprintable

class ActionInspector(PPrintable):
    '''
    All Actions have:
    - option_strings
    - dest
    - nargs
    - const
    - default
    - type
    - choices
    - required
    - help
    - metavar

    argparse._VersionAction adds:
    - version

    argparse._SubParsersAction adds:
    - prog
    - parser_class
    '''

    NAME_MAP = {
        argparse._StoreAction : 'store',
        argparse._StoreConstAction : 'store_const',
        argparse._StoreTrueAction : 'store_true',
        argparse._StoreFalseAction : 'store_false',
        argparse._AppendAction : 'append',
        argparse._AppendConstAction : 'append_const',
        argparse._CountAction : 'count',
        #argparse._HelpAction : 'help',
        argparse._VersionAction : 'version',
        argparse._SubParsersAction : 'parsers',
        argparse._ExtendAction : 'extend',
        }

    def __init__(self, action):
        if action.__class__ in self.NAME_MAP:
            self.action_type = self.NAME_MAP[action.__class__]
        else:
            self.action_type = action.__class__.__name__

        # passthrough of attributes
        self.action = action
        self.option_strings = action.option_strings
        self.dest = action.dest
        self.nargs = action.nargs
        self.const = action.const
        self.default = action.default
        self.type = action.type
        self.choices = action.choices
        if is_subparser_action(action):
            self.choices = {key: ParserInspector(val)
                            for key, val in action.choices.items()}
        self.required = action.required
        self.help = action.help
        self.metavar = action.metavar

        # optional attributes
        for attr in ('version', 'prog', 'parser_class'):
            if hasattr(action, attr):
                setattr(self, attr, getattr(action, attr))
            else:
                setattr(self, attr, None)

        # introspection

    def _get_kwargs(self):
        return [(name, getattr(self, name)) for name in [
            'option_strings',
            'action_type',
            'dest',
            'nargs',
            'const',
            'default',
            'type',
            'choices',
            'required',
            'help',
            'metavar',
            'version',
            'prog',
            'parser_class',
            ]]

class ParserInspector(PPrintable):
    'Introspection on an argparse.ArgumentParser'

    # class Action:
    # - option_strings
    # - dest
    # - nargs=None
    # - const=None
    # - default=None
    # - type=None
    # - choices=None
    # - required=False
    # - help=None
    # - metavar=None

    # class _StoreAction: (action='store', action=None)

    # class _StoreConstAction: (action='store_const')
    # - Required:
    #   - const
    # - Removed:
    #   - nargs
    #   - type
    #   - choices
    # - Fixed:
    #   - nargs=0

    # class _StoreTrueAction: (action='store_true')
    # - Changed:
    #   - default=False
    # - Removed:
    #   - nargs
    #   - const
    #   - type
    #   - choices
    #   - metavar
    # - Fixed:
    #   - const=True

    # class _StoreFalseAction: (action='store_false')
    # - Changed:
    #   - default=True
    # - Removed:
    #   - nargs
    #   - const
    #   - type
    #   - choices
    #   - metavar
    # - Fixed:
    #   - const=True

    # class _AppendAction: (action='append')

    # class _AppendConstAction: (action='append_const')
    # - Required:
    #   - const
    # - Removed:
    #   - type
    #   - choices

    # class _CountAction: (action='count')
    # - Removed:
    #   - nargs
    #   - const
    #   - type
    #   - choices
    #   - metavar
    # - Fixed:
    #   - nargs=0

    # class _HelpAction: (action='help')
    # - Changed:
    #   - dest=SUPPRESS
    #   - default=SUPPRESS
    # - Removed:
    #   - nargs
    #   - const
    #   - type
    #   - choices
    #   - required
    #   - metavar
    # - Fixed:
    #   - nargs=0

    # class _VersionAction: (action='version')
    # - Changed:
    #   - dest=SUPPRESS
    #   - default=SUPPRESS
    # - Added:
    #   - version=None
    # - Removed:
    #   - nargs
    #   - const
    #   - type
    #   - choices
    #   - required
    #   - metavar
    # - Fixed:
    #   - nargs=0

    # class _SubParsersAction: (action='parsers')
    # - Added:
    #   - prog
    #   - parser_class
    # - Changed:
    #   - dest=SUPPRESS
    # - Removed:
    #   - nargs
    #   - const
    #   - default
    #   - type
    #   - choices
    # - Fixed:
    #   - nargs=PARSER
    #   - choices=self._name_parser_map

    # class _ExtendAction: (action='extend')


    def __init__(self, parser):
        self.parser = parser
        self.option_actions = [
            ActionInspector(a) for a in self.parser._optionals._group_actions]
        self.position_actions = [
            ActionInspector(a) for a in self.parser._positionals._group_actions
            if not is_subparser_action(a)]
        self.subparser_actions = []
        if self.parser._subparsers:
            self.subparser_actions = [
                ActionInspector(a) for a in self.parser._subparsers._group_actions
                if is_subparser_action(a)]
        self.actions = self.option_actions + self.position_actions + \
                       self.subparser_actions

        self.option_strings = sum(
            [a.option_strings for a in self.option_actions], start=[])

    def _get_kwargs(self):
        return [(name, getattr(self, name)) for name in [
            'option_strings',
            'option_actions',
            'position_actions',
            'subparser_actions',
            ]]

class TestArgparse_Bar(unittest.TestCase):
    BAR_PROG = 'bar'
    BAR_BASH_COMPLETION = os.path.join(SCRIPT_DIR, '..', 'completions', 'bar')

    def assertEqualCompletion(self, args, expected_completions):
        'Asserts that the expected completions are obtained'
        actual = get_completion(self.BAR_BASH_COMPLETION, self.BAR_PROG, args)
        self.assertEqual(set(expected_completions), set(actual))

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
