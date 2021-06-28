#!/usr/bin/env python3
'''
Represents a tic-tac-toe game
'''

from collections import namedtuple
from dataclasses import dataclass
from enum import IntEnum

class Player(IntEnum):
    EMPTY = 0
    P1 = 1
    P2 = 2

class TicTacToe:
    '''
    Represents an instance of a tic-tac-toe game.

    There is a start_state which is the initial state of the game.

    A game state is handled internally, but for those who care, it is
    represented as a tuple of 11 values, each of which is a Player value:

    - state[0]: who's turn it is
    - state[1-9]: the board laid out in row-major order starting from the top
    - state[10]: the winner
    '''

    def __init__(self):
        self.start_state = (Player.P1,) + (Player.EMPTY,) * 10

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

    def available_actions(self, state):
        'Returns available actions by the current player in the current state'
        if state[0] == Player.EMPTY:  # game is over
            return []
        return [i for i in range(1, 10) if state[i] == Player.EMPTY]

    def move(self, state, action):
        '''
        Give the resultant state after the current player does the given
        action.
        '''
        assert state[action] == Player.EMPTY
        assert state[0] != Player.EMPTY, 'cannot move if the game is over'
        assert state[10] == Player.EMPTY, 'cannot move if the game is over'

        p = state[0] # current player

        # check to see if we have a winner
        newstate = list(state)
        if self._is_winning_move(state, action):
            newstate[0] = Player.EMPTY
            newstate[10] = p
        else:
            newstate[0] = Player.P2 if state[0] == Player.P1 else Player.P1

        # fill in that square
        newstate[action] = p

        # if the board is full, mark it as a tie
        if not any(x == Player.EMPTY for x in newstate[1:10]):
            newstate[0] = Player.EMPTY
            newstate[10] = Player.EMPTY

        return tuple(newstate)

    def _is_winning_move(self, state, action):
        '''
        Returns true if the action is a winning move for the current player

        @param state: a state tuple
        @param action: index into the state tuple for the current player
        '''
        assert state[action] == Player.EMPTY
        assert state[0] != Player.EMPTY, 'cannot move if the game is over'
        assert state[10] == Player.EMPTY, 'cannot move if the game is over'

        p = state[0] # current player
        s = state

        to_check = {
            1 : ((2, 3), (4, 7), (5, 9)),
            2 : ((1, 3), (5, 8)),
            3 : ((1, 2), (6, 9), (5, 7)),
            4 : ((5, 6), (1, 7)),
            5 : ((1, 9), (2, 8), (3, 7), (4, 6)),
            6 : ((3, 9), (4, 5)),
            7 : ((1, 4), (8, 9), (5, 3)),
            8 : ((7, 9), (2, 5)),
            9 : ((7, 8), (6, 3), (1, 5)),
            }

        return any(p == s[x[0]] and p == s[x[1]]
                   for x in to_check[action])

