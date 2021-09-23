#!/usr/bin/env python3
'''
Removes all comments and docstrings from a given python file.

This is done by parsing the python file into an Abstract Syntax Tree (AST),
removing docstrings, then converting it back to code.
'''

import ast
import argparse
import sys

def populate_parser(parser=None):
    'Populate the parser with command-line arguments, or create one'
    if not parser:
        parser = argparse.ArgumentParser()
    parser.description = sys.modules[__name__].__doc__
    parser.add_argument('python_file')
    parser.add_argument('output', nargs='?')
    return parser

def empty_func():
    'empty function'

def other_empty_func():
    # no docstring here, so needs to have 'pass'
    pass

def read(fname):
    with open(fname, 'r') as fin:
        return fin.read()

def write(fname, contents):
    with open(fname, 'w') as fout:
        fout.write(contents)

def remove_docstrings(node):
    'Removes docstrings'
    relevant_types = (
        ast.FunctionDef,
        ast.ClassDef,
        ast.AsyncFunctionDef,
        ast.Module,
        )
    for child in ast.walk(node):
        # narrow down the search until we know we have a node with a docstring
        if not isinstance(child, relevant_types): continue
        if not child.body: continue
        candidate = child.body[0]
        if not isinstance(candidate, ast.Expr): continue
        if not hasattr(candidate, 'value'): continue
        if not isinstance(candidate.value, ast.Str): continue

        # if we get here, then candidate is a docstring.  Let's remove it
        del child.body[0]

        # if that was the only element and this was not a module, then we need
        # to add a pass
        if not child.body and not isinstance(child, ast.Module):
            child.body.append(ast.Pass())

def main(arguments):
    '''
    Main logic here
    '''
    # parse arguments
    parser = populate_parser()
    args = parser.parse_args(arguments)
    print(args)

    contents = read(args.python_file)
    print(f'Original file length: {len(contents)}')

    parsed = ast.parse(contents)
    remove_docstrings(parsed)

    new_contents = ast.unparse(parsed)
    print(f'New length: {len(new_contents)}')

    if args.output:
        write(args.output, new_contents)
    else:
        print(new_contents)

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
