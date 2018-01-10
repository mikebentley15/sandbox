import argparse
import sys

def main(arguments):
    parser = argparse.ArgumentParser()
    parser.add_argument('-O0', action='store_const', dest='optl', const=0)
    parser.add_argument('-O1', action='store_const', dest='optl', const=1)
    parser.add_argument('-O2', action='store_const', dest='optl', const=2)
    parser.add_argument('-O3', action='store_const', dest='optl', const=3)
    args = parser.parse_args(arguments)
    print(args.__repr__())
    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
