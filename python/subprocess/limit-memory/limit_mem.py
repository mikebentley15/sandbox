#!/usr/bin/env python3

import resource
import subprocess as subp
import sys

if __name__ == '__main__':
    if '-h' in sys.argv or '--help' in sys.argv:
        print('Usage: limit_mem.py <mb-count> <command>')
        sys.exit(0)
    byte_count = int(sys.argv[1]) * 1024 * 1024
    command = sys.argv[2:]
    
    def set_limit():
        'set limits'
        try:
            resource.setrlimit(resource.RLIMIT_DATA, (byte_count, byte_count))
        except ValueError:
            pass

    try:
        subp.check_call(command, preexec_fn=set_limit)
    except subp.SubprocessError:
        print(f'Exceeded memory usage of {byte_count}')
        sys.exit(1)
    else:
        print(f'Stayed within memory usage of {byte_count}')
