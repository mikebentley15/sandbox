#!/usr/bin/env python3

'Simple server that listens and echos back any sent data'

import argparse
from socket import socket, AF_INET, SOCK_STREAM
import sys

# AF_INET: means IPv4
# SOCK_STREAM: means TCP

def parse_args(arguments):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description='create an echer server',
        )
    parser.add_argument('-p', '--port', type=int, default=65432,
                        help='Port to listen on.')
    parser.add_argument('--ip', default='', type=str,
                        help='''
                            Which addresses can connect.  Use 127.0.0.1 for
                            only local connections, a specific IP address or
                            hostname to restrict to a single external machine,
                            or leave it empty to accept from anywhere.
                            ''')
    return parser.parse_args(arguments)

def listen(host, port):
    with socket(AF_INET, SOCK_STREAM) as s:
        s.bind((host, port))
        print('listening on port', port)
        s.listen()
        conn, addr = s.accept()
        with conn:
            print('Connected to', addr)

            # return everything received
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                print('Received', repr(data))
                conn.sendall(data)

def main(arguments):
    args = parse_args(arguments)
    listen(args.ip, args.port)
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
