#!/usr/bin/env python3

import tabulate

words = [
    '17Again.mp4',
    '21.mp4',
    'BadSeed_The.mp4',
    'BatmanBegins.mp4',
    'BetterOffDead.mp4',
    'Big.mp4',
    'BigSleep_The.mp4',
    'BlackSheep.mp4',
    'BourneIdentity_The.mp4',
    'BourneUltimatum_The.mp4',
    'Breach.mp4',
    'CaineMutiny_The.mp4',
    'Casablanca.mp4',
    'CasinoRoyale.mp4',
    'CatchMeIfYouCan.mp4',
    'ChristmasStory_A.mp4',
    'DarkNight_The.mp4',
    'DavidCopperfield.mp4',
    'DialMForMurder.mp4',
    'Dodgeball.mp4',
    'DoubleIndemnity.mp4',
    'FerrisBuelersDayOff.mp4',
    'FindingNeverland.mp4',
    'ForestGump.mp4',
    'Gaslight.mp4',
    'George.mp4',
    'Ghostbusters.mp4',
    'Gravity.mp4',
    'GreyGardens.mp4',
    'GroundhogDay.mp4',
    'GulliversTravels.mp4',
    'JackAndTheBeanstalk.mp4',
    'MarkTwainsToughingIt-Night1.mp4',
    'MarkTwainsToughingIt-Night2.mp4',
    ]

table = tabulate.tabulate(words, tabulate.terminal_width())
print('\n'.join(table))