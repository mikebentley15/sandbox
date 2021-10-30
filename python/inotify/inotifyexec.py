#!/usr/bin/env python3
'Use inotify-tools to watch a directory or files and execute a command'

__author__ = 'Michael Bentley'

import subprocess as subp
import sys


def popen_reader(*args, stdout=subp.PIPE, **kwargs):
    '''Creates a pipe (on stdout by default) and returns a line generator

    Example:

        for line in popen_reader(['ls', '-ltr'], text=True):
            line = line[:-1]            # remove newline
            pieces = line.split()       # split into columns
            print(pieces[0], pieces[4], pieces[-1]) # subset of columns

    Defaults the stdout=subprocess.PIPE instead of stdout=None.  You can change
    this and/or set stderr=subprocess.PIPE to include that in the line output
    to be received.

    This is different from subprocess.check_output() in that you can process
    the output while the process is still running.  So this function can be
    used for long-running programs or even programs that may never end.  It's a
    good way to add asynchronous execution to your python program.
    '''
    with subp.Popen(*args, stdout=stdout, **kwargs) as proc:
        for line in proc.stdout:
            yield line

def inotifywait(args):
    '''Used in a for statement like so

        files = glob.glob('*.cpp')
        for file, types in inotifywait(['--monitor'] + files):
            # do something

    The args will be given to the `inotifywait` command-line tool from
    the inotify-tools, so call `inotifywait --help` to see what the args
    are.

    If you want to use this in a loop like above for multiple file events, use
    the `-m` or `--monitor` flag so that it doesn't exit on the first event.

    This command will not work appropriately if you use the `-d` or `--daemon`
    flag since this command parses the command-line args.

    This automatically adds the --recursive and --quiet flags, so you can pass
    files or directories as args.  If you passed a filename instead of a
    directory, the filename returned will be None when the event happens.

    Returns a tuple of two values:
    - fname (str): filename (or directory) that triggered the event
    - types list(str): list of types of events that triggered
      For example: ['OPEN', 'ISDIR']
    '''
    command = [
        'inotifywait',
        '--recursive',          # allow for directories
        '--format', '%w%f %e',  # specific format of output
        '--quiet',              # suppress info at the start
    ]
    command.extend(args)
    for line in popen_reader(command, text=True):
        split = line.rsplit(maxsplit=1)
        types = split[-1].split(',')
        fname = split[0] if len(split) == 2 else None
        yield (fname, types)

def inotifyexec(args, command, filter=None, **kwargs):
    '''Pass args to inotifywait, and run the given command subprocess

    The command will be given to subprocess.check_call() along with any extra
    forwarded keyword arguments.

    You may provide a filter function that takes two arguments, the filepath
    and this list of event types.  Note that the filepath will be None if it
    was given in args not inside of a directory (see inotifywait()).  By
    default, the command will be called regardless of the type of triggered
    event or the directory or file where the event took place.
    '''
    if filter is None:
        filter = lambda fname, types: True  # dummy function
    for fname, types in inotifywait(args):
        if filter(fname, types):
            subp.check_call(command, **kwargs)

def main(arguments=sys.argv[1:]):
    if '-h' in arguments or '--help' in arguments:
        oldhelp = subp.run(
            ['inotifywait', '--help'],
            stdout=subp.PIPE,
            check=False,
            text=True,
        )
        print('inotifyexec: a wrapper around inotifywait to run a command')
        print('Usage: inotifyexec <inotifywait-args> -- <inotifyexec-args>')
        print('inotifyexec-args:')
        print('  command    Any arguments not given to flags are interpreted')
        print('             as arguments for a bash command')
        print()
        print(oldhelp.stdout)
        return 0

    if '--' not in arguments:
        print('Error: you must use "--" to indicate the start of the'
              ' inotifyexec arguments', file=sys.stderr)
        return 1

    idx = arguments.index('--')
    args = arguments[:idx]
    command = arguments[idx+1:]
    if not command:
        print('Error: you must specify a command to run on an event',
              file=sys.stderr)
        return 1

    def filter(fname, types):
        print(f'{fname} changed: (types = {types})')
        return True

    return inotifyexec(args, command, filter=filter)

if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
