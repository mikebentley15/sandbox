#!/usr/bin/env python3

import argparse
import re
import sys
import textwrap

header = r'''\documentclass{article}
\usepackage[margin=0.7in]{geometry}

\newcommand{\timespan}[2]{\textbf{#1 to #2}}
\newcommand{\datepage}[1]{\newpage\section*{\centering\Huge{#1}}}
\newcommand{\zoom}{\textbf{\textit{ZOOM}\quad}}

\begin{document}

\fontsize{16pt}{20pt}\selectfont

'''

date_matcher = re.compile(r'(Monday|Tuesday|Wednesday|Thursday|Friday)')
time_pattern = r'(\d+:\d+) to (\d+:\d+) [-\s]*(.*)$'
zoom_matcher = re.compile(r'Zoom ')
time_matcher = re.compile(time_pattern)
empty_matcher = re.compile(r'^\s$')

def parse_args(arguments):
    parser = argparse.ArgumentParser(
        description="Convert Lucy's 3rd grade schedule to a LaTeX file")
    parser.add_argument('raw_text', help='Raw text of schedule from her teacher')
    parser.add_argument('-o', '--output', default='lucy-schedule.tex',
                        help='output file')
    args = parser.parse_args(arguments)
    return args


def main(arguments):
    args = parse_args(arguments)
    with open(args.raw_text, 'r') as fin:
        print('writing', args.output)
        with open(args.output, 'w') as fout:
            fout.write(header)
            for line in fin:
                zoom_match = zoom_matcher.match(line)
                time_match = time_matcher.match(line)
                if date_matcher.match(line):
                    fout.write(r'\datepage{' + line.strip() + '}\n')
                    fout.write('\\begin{description}\n')
                elif zoom_match:
                    time_find = time_matcher.search(line)
                    fout.write(r'  \item[\timespan{' + time_find.group(1)
                               + '}{' + time_find.group(2) + '}] - \\zoom\n')
                    fout.write('    ' + '\n    '.join(
                               textwrap.wrap(time_find.group(3), width=76)))
                    fout.write('\n')
                elif time_match:
                    fout.write(r'  \item[\timespan{' + time_match.group(1)
                               + '}{' + time_match.group(2) + '}] -\n')
                    fout.write('    ' + '\n    '.join(
                               textwrap.wrap(time_match.group(3), width=76)))
                    fout.write('\n')
                elif empty_matcher.match(line):
                    fout.write('\\end{description}\n')
                    fout.write('\n')
            fout.write(r'\end{document}')


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
