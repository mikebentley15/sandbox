#!/usr/bin/env python3
'''
Classes for representing the playing deck state for No Stress Chess.  If you
call this script as a program, then it imitates drawing from the deck using a
command-line interface.

Author:   Michael Bentley
Date:     03 June 2021
'''

from enum import Enum
import random
import sys

class CardType(Enum):
    NONE     = 0  # no card
    KING     = 1  # move king
    QUEEN    = 2  # move queen
    ROOK     = 3  # move rook
    BISHOP   = 4  # move bishop
    KNIGHT   = 5  # move knight
    PAWN     = 6  # move pawn
    PREVIOUS = 7  # use previous card in either pile

    def __str__(self):
        if self == CardType.NONE:       return 'NONE'
        elif self == CardType.KING:     return 'KING'
        elif self == CardType.QUEEN:    return 'QUEEN'
        elif self == CardType.ROOK:     return 'ROOK'
        elif self == CardType.BISHOP:   return 'BISHOP'
        elif self == CardType.KNIGHT:   return 'KNIGHT'
        elif self == CardType.PAWN:     return 'PAWN'
        elif self == CardType.PREVIOUS: return 'PREVIOUS'

class Pile:
    '''
    Represents a pile of cards (as in a stack).  Add a card by appending to
    self.cards list with self.cards.append(<card-type>).  Remove a card by
    removing from the end of self.cards with self.cards.pop().
    '''

    def __init__(self):
        self.cards = []

    def previous(self):
        'Returns the non-PREVIOUS card from the top of this pile'
        for card in reversed(self.cards):
            if card != CardType.PREVIOUS:
                return card
        else:
            return CardType.NONE

    def shuffle(self):
        'shuffle the pile'
        random.shuffle(self.cards)


class GameDeckState:
    '''
    Maintains the state of the deck of cards (using Pile) and each player's
    piles.
    '''

    def __init__(self):
        self.draw_pile = Pile()
        self.player_1_discard = Pile()
        self.player_2_discard = Pile()
        self.restart()

    def restart(self):
        '''
        Restarts, emptying the discard piles and restocking and shuffling the
        draw pile.
        '''
        self.player_1_discard.cards.clear()
        self.player_2_discard.cards.clear()
        # populate the draw pile
        self.draw_pile.cards = (
                    [CardType.KING]     *  7 +
                    [CardType.QUEEN]    *  7 +
                    [CardType.ROOK]     *  8 +
                    [CardType.BISHOP]   *  8 +
                    [CardType.KNIGHT]   *  8 +
                    [CardType.PAWN]     * 11 +
                    [CardType.PREVIOUS] *  6
                )
        self.draw_pile.shuffle()

    def draw(self, player_number):
        'Draw the card for the given player'
        if player_number == 1:
            discard = self.player_1_discard
        elif player_number == 2:
            discard = self.player_2_discard
        else:
            raise ValueError('Player number must be 1 or 2')
        try:
            card = self.draw_pile.cards.pop()
            discard.cards.append(card)
            return card
        except IndexError:
            print()
            print('Draw pile empty, reshuffling')
            print()
            self.restart()
            return self.draw(player_number)

    def draw_p1(self):
        'Draw the card for player 1'
        self.draw(1)

    def draw_p2(self):
        'Draw the card for player 2'
        self.draw(2)

def main(arguments):
    'main logic'
    game = GameDeckState()
    print('Welcome to No Stress Chess!')
    print()
    print('Press Enter when you are ready to draw a card')
    print()

    current_player = 1
    other_player = 2
    while True:
        if current_player == 1:
            print()

        input(f'player {current_player}: ')
        card = game.draw(current_player)
        print(f'  {card}')

        # handle special card cases
        if card == CardType.NONE:
            raise RuntimeError('No card type')
        elif card == CardType.PREVIOUS:
            print('  Choices:')
            print('   ', game.player_1_discard.previous())
            print('   ', game.player_2_discard.previous())

        # swap players
        current_player, other_player = other_player, current_player

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
