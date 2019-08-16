#!/usr/bin/env python3

'Simple client that sends a small message via TCP'

import argparse
from socket import socket, AF_INET, SOCK_STREAM, timeout as Timeout
import sys

# AF_INET: means IPv4
# SOCK_STREAM: means TCP

def parse_args(arguments):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description='send a message via TCP and print the reply',
        )
    parser.add_argument('-p', '--port', type=int, default=65432,
                        help='Port to connect to.')
    parser.add_argument('-t', '--timeout', type=float, default=1.5,
                        help='''
                            Timeout.  Set to a negative number to disable
                            timeout.  Set to 0 to have an instant timeout.
                            ''')
    parser.add_argument('host', default='127.0.0.1', nargs='?',
                        help='ip or hostname of destination')
    parser.add_argument('message', default='Hello world!', nargs='?',
                        help='what to send')
    args = parser.parse_args(arguments)
    if args.timeout < 0:
        args.timeout = None
    return args

def send(host, port, message, timeout=1.5):
    with socket(AF_INET, SOCK_STREAM) as s:
        s.settimeout(timeout)
        s.connect((host, port))
        print(f'sending "{message}" to {host}:{port}')
        s.sendall(message.encode('utf-8'))
        data = s.recv(1024)
    print('Received', repr(data))

def main(arguments):
    args = parse_args(arguments)
    try:
        send(args.host, args.port, args.message, timeout=args.timeout)
    except ConnectionRefusedError:
        print('Connection refused', file=sys.stderr)
        return 1
    except Timeout:
        print('Connection timed out', file=sys.stderr)
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
