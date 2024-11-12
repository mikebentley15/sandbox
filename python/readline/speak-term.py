#!/usr/bin/env python3

import atexit
import os
import readline
import subprocess as subp
import cmd
import sys

from CommandWrapPrompt import CommandWrapPrompt

def speak(arguments, text):
    subp.run(['speak-ng'] + arguments + [text], check=True)

def readline_setup_history(histfile, length=1000):
    try:
        readline.read_history_file(histfile)
        readline.set_history_length(length)
    except FileNotFoundError:
        pass
    atexit.register(readline.write_history_file, histfile)

def main(arguments):
    histfile = os.path.join(os.path.expanduser('~'), '.speakterm_history')
    readline_setup_history(histfile)
    command = lambda line: speak(arguments, line)
    prompt = CommandWrapPrompt(command)
    prompt.intro = (
        '\n'
        'Speak your mind through your computer, easily!\n'
        '- Simply type in what you want the computer to say.\n'
        '- Press CTRL-D to exit.\n'
    )
    prompt.prompt = 'speak> '
    prompt.postloop = lambda: command('Goodbye')
    prompt.cmdloop()

if __name__ == '__main__':
    main(sys.argv[1:])
