#!/usr/bin/env python3
'TODO: Description here' # FIXME

import argparse
import sys
import logging
from pathlib import Path

def populate_parser(parser=None):
    'Create an argument parser (or populate an existing one)'
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.formatter_class = argparse.ArgumentDefaultsHelpFormatter
    parser.description = 'TODO: Description here' # FIXME
    parser.add_argument('-l', '--loglevel', default='INFO',
                        help='Set logging level to this (from logging module)')
    return parser

def parse_args(parser, arguments):
    'Parse arguments using parser + post-processing and checks'
    args = parser.parse_args(arguments)
    args.loglevel = getattr(logging, args.loglevel.upper())
    # TODO: extra post-processing or checks
    return args

def main(arguments):
    'Main logic here including argument parsing'
    parser = populate_parser()
    args = parse_args(parser, arguments)
    logging.basicConfig(level=args.loglevel)
    logging.debug(f'Arguments: {arguments}')
    logging.debug(f'Parsed args: {args}')
    # TODO: main logic here

if __name__ == '__main__':
    main(sys.argv[1:])
