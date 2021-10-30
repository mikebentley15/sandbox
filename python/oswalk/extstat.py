#!/usr/bin/env python3

import collections
import os
import sys

def listdir_rec(dir='.', sort=False):
    '''Returns a generator of recursive directory listings

    Item returned from the generator is a 2-tuple:
    - path (str)
    - type (str): one of ('d', 'f') for directory or file, respectively
    '''
    for root, dirs, files in os.walk(dir):
        if sort:
            dirs.sort()
            files.sort()
        for dname in dirs:
            yield (os.path.join(root, dname), 'd')
        for fname in files:
            yield (os.path.join(root, fname), 'f')

def findext(dir='.'):
    'Generator to recursively search dir for file extensions'
    for fname, ftype in listdir_rec(dir, sort=False):
        if ftype == 'f':
            _, ext = os.path.splitext(fname)
            yield ext

def extstat(dir='.'):
    'Returns a collection.Counter of found file extensions in dir'
    return collections.Counter(findext(dir))

def main(arguments=None):
    if '-h' in arguments or '--help' in arguments:
        print('Usage:  extstat [<dir>] ...')
        print()
        print('  Prints statistics on all of the file extensions in the given')
        print('  directory.  If the directory is omitted, uses the current')
        print('  directory.  If a file is given, a warning is printed.')
        return 0
    if not arguments:
        arguments = ['.']
    for path in arguments:
        if not os.path.isdir(path):
            print(f'Warning: not a directory: "{path}"', file=sys.stderr)
            continue
        c = extstat(path)
        if not path.endswith('/'):
            path += '/'
        print(path)
        if not c:
            continue
        largest_ext = max(max(len(x) for x in c), len('<empty>'))
        fstr = f'    {{:<{largest_ext}s}}  {{}}'
        for ext, n in c.most_common():
            if not ext:
                ext = '<empty>'
            print(fstr.format(ext, n))
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
