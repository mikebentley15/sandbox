from typing import Optional
from argparse import ArgumentParser
import cppyy
import sys

def populate_parser(parser: Optional[ArgumentParser] = None):
    if parser is None:
        parser = ArgumentParser()
    return parser

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)
    print(arguments)
    ...
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
