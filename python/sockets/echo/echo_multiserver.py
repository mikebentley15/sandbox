#!/usr/bin/env python3

'Simple server that listens and echos back any sent data'

import argparse
from select import select
from socket import socket, AF_INET, SOCK_STREAM
import sys

# AF_INET: means IPv4
# SOCK_STREAM: means TCP

def parse_args(arguments):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        description='create an echer server',
        )
    parser.add_argument('-p', '--ports', type=str, default='8000:8100',
                        help='''
                            Port range to listen on, specified with a colon.
                            Both bounds are inclusive.
                            ''')
    parser.add_argument('--ip', default='', type=str,
                        help='''
                            Which addresses can connect.  Use 127.0.0.1 for
                            only local connections, a specific IP address or
                            hostname to restrict to a single external machine,
                            or leave it empty to accept from anywhere.
                            ''')
    return parser.parse_args(arguments)

def create_socket(host, port):
    s = socket(AF_INET, SOCK_STREAM)
    try:
        s.bind((host, port))
        s.listen()
    except OSError as ex:
        print('Failed to bind to port', port, ':', ex)
        s.close()
        return None
    else:
        return s

def listen(host, ports):
    server_sockets = [create_socket(host, port) for port in ports]
    server_sockets = [x for x in server_sockets if x is not None]
    all_sockets = list(server_sockets)
    while True:
        readable, _, _ = select(all_sockets, [], [])
        for s in readable:
            if s in server_sockets:
                connected_client, address = s.accept()
                print('Client connected to', address,
                      'from port', s.getsockname()[1])
                all_sockets.append(connected_client)
            else:
                data = s.recv(1024)
                if not data:
                    s.close()
                    all_sockets.remove(s)
                    print('Client disconnected')
                else:
                    print('Received', repr(data))
                    s.sendall(data) # send it back

def main(arguments):
    args = parse_args(arguments)
    port_split = args.ports.split(':')
    args.ports = range(int(port_split[0]), int(port_split[1]) + 1)
    print(f'Attempting to listen to ports {port_split[0]} to {port_split[1]}')
    listen(args.ip, args.ports)
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
