#!/usr/bin/env python3
'''
Represents a tic-tac-toe game
'''

from collections import namedtuple
from dataclasses import dataclass
from enum import IntEnum

GameState = namedtuple('GameState', 'player,x11,x12,x13,x21,x22,x23,x31,x32,x33')

class Player(IntEnum):
    EMPTY = 0
    P1 = 1
    P2 = 2

class TicTacToe:
    def __init__(self):
        self.start_state = tuple([Player.EMPTY] * 11)

    def equivalent_states(self, state):
        '''
        Generates the list of all states that are symmetrically equivalent to
        the given one.
        '''
        class Ops(Enum):
            ROT = 1
            VMIR = 2
            HMIR = 3
        equiv = set()
        frontier = [(state, '')]

        def push(s, p):
            '''only add to the frontier if it's a new state'''
            if s not in equiv:
                frontier.append((s, p))

        # I think this is right...
        while frontier:
            current, path = frontier.pop()
            if current in equiv: continue  # skip
            #print(f'state: {current}, path: {path}')
            equiv.add(current)
            push(self._rot_counterclockwise(current), path + 'r')
            push(self._mir_horizontal(current), path + 'h')

        return list(equiv)

    def _rot_counterclockwise(self, state):
        '''
        Takes the state and returns a copy that is rotated counter-clockwise by
        90 degrees.
        '''
        return state[:1] + state[3:10:3] + state[2:10:3] + state[1:10:3] + state[10:]

    def _mir_horizontal(self, state):
        '''
        Takes the state and returns a copy that is mirrored midway across the
        horizontal centerline.
        '''
        return state[:1] + state[7:10] + state[4:7] + state[1:4] + state[10:]

    def print_state(self, state):
        c = lambda x: ' ' if x == Player.EMPTY else \
                     ('O' if x == Player.P1 else 'X')
        print()
        print(f'  {c(state[1])} | {c(state[2])} | {c(state[3])}')
        print( ' ---+---+---')
        print(f'  {c(state[4])} | {c(state[5])} | {c(state[6])}')
        print( ' ---+---+---')
        print(f'  {c(state[7])} | {c(state[8])} | {c(state[9])}')
        print()

    def canonicalize(self, state):
        '''
        Returns the canonical version of the state.  i.e., the one version of
        the equivalent states that is considered the official one.
        '''
        return max(self.equivalent_states(state))
