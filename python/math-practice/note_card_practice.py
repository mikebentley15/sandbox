#!/usr/bin/env python3

'''
To randomly show one line at a time from a note card CSV file with columns
'question' and 'answer'.
'''

import argparse
import csv
import random
import sys

def populate_parser(parser=None):
    'Populate argument parser'
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = '''
        To randomly show one line at a time from a note card CSV file with
        columns 'question' and 'answer'.  Waits for the user to answer and
        gives feedback on if they were right and if not what the correct answer
        is.
        '''

    parser.add_argument('-N', '--number', type=int, default=10,
                        help='number of random cards to show')
    parser.add_argument('csv', help='CSV file containing card info',
                        type=argparse.FileType('r'))

    return parser

class NoteCard:
    def __init__(self, question, answer):
        self.question = question
        self.answer = answer

def main(arguments):
    parser = populate_parser()
    args = parser.parse_args(arguments)

    reader = csv.DictReader(args.csv)
    cards = [NoteCard(row['question'], row['answer']) for row in reader]

    for i in range(args.number):
        card = random.choice(cards)
        answer = input(f'{i+1}.  {card.question} ')
        if answer != card.answer:
            print('  sorry, the correct answer is: ', card.answer)
        else:
            print('  correct!')
        print()

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
