'A generator to return each line from stdout'

import subprocess as subp
import tempfile

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

def main():
    contents = 'a\nb\ncde\nfg\nhi\n\njkl'
    print('Creating a file with the following contents within quotes:')
    print()
    print(f'"{contents}"')
    print()
    print('Then we will call "cat" on that file, outputting each line one at a'
          'time from python (if everything works according to plan).  Each'
          'line will be prefixed with the output line number')
    print()
    with tempfile.NamedTemporaryFile('w') as ftmp:
        print('Creating file', ftmp.name)
        print()
        ftmp.write('a\nb\ncde\nfg\nhi\n\njkl')
        ftmp.flush()
        reader = popen_reader(['cat', ftmp.name], text=True)
        for i, line in enumerate(reader):
            print(f'{i}: {line}', end='')

if __name__ == '__main__':
    main()
