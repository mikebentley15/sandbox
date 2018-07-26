'''
This script logic was copied from:
    https://github.com/pipalacademy/jupyterhub-simple/blob/master/src/add-users.py
but then modified by Michael Bentley

Note: this script should be run as root since it calls useradd
'''

import sys
import crypt
import os
import argparse
import subprocess as subp
import pwd

def get_users(usersfile):
    users = []
    with open(usersfile, 'r') as infile:
        for line in infile:
            try:
                username, pw = line.strip().split(':', 1)
            except:
                print('Warning: found line with no ":" mark')
            else:
                users.append((username, pw))
    return users

def add_users(users):
    '''
    Adds the users to the system.
    
    >>> add_users([('michael', 'mikepass'), ('bob', 'bobs-password')])
    Added user michael
    Added user bob
    '''
    for username, pw in users:
        cryptpw = crypt.crypt(pw)
        try:
            pwd.getpwnam(username)
        except KeyError:
            subp.check_call([
                'useradd',
                '--create-home',
                '--password', cryptpw,
                username
                ])
            subp.check_call([
                'chown',
                '-R',
                '{0}:{0}'.format(username),
                '/home/{0}'.format(username),
                ])
            print('Added user', username)
        else:
            print('User', username, 'already exists')
            

def main(arguments):
    parser = argparse.ArgumentParser()
    parser.add_argument('usersfile')
    args = parser.parse_args(arguments)

    if not os.path.exists(args.usersfile):
        print(args.usersfile, 'not found', file=sys.stderr)
        return

    add_users(get_users(args.usersfile))

if __name__ == '__main__':
    main(sys.argv[1:])
        
