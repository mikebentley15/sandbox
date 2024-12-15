#!/usr/bin/env python3

from __future__ import annotations
import argparse
import sys
from pathlib import Path
from functools import reduce

def populate_parser(parser: argparse.ArgumentParser | None = None) -> argparse.ArgumentParser:
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''Part 1 solution'''
    parser.add_argument('input_file', type=Path)
    return parser

def pairwise(iterator):
    prev = next(iterator)
    for val in iterator:
        yield (prev, val)
        prev = val

def report_pair_stats(a, b):
    is_distance_within_three = abs(a - b) < 4
    is_increasing = a < b
    is_decreasing = a > b
    return (is_distance_within_three, is_increasing, is_decreasing)

def tup_and(a, b):
    return tuple(x and y for (x, y) in zip(a, b))

def is_report_valid(report):
    pair_stats = (report_pair_stats(a, b) for a, b in pairwise(report))
    is_good_distance, is_increasing, is_decreasing = reduce(tup_and, pair_stats, (True, True, True))
    return is_good_distance and (is_increasing or is_decreasing)

def main(arguments: list[str]) -> int:
    parser = populate_parser()
    args = parser.parse_args(arguments)
    with args.input_file.open('r') as fin:
        reports = ((int(x) for x in line.split()) for line in fin)
        valid_report_count = sum(is_report_valid(report) for report in reports)
    print(f'Valid Report Count: {valid_report_count}')
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
