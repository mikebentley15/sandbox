import get_caller

print('Global Level:')
print(' ', get_caller.get_caller_info())
print(' ', get_caller.get_caller_file())
print()

def main():
    print('From Function Level:')
    print(' ', get_caller.get_caller_info())
    print(' ', get_caller.get_caller_file())
    print()

    print('From Lambda:')
    get_info = lambda: get_caller.get_caller_info()
    get_file = lambda: get_caller.get_caller_file()
    print(' ', get_info())
    print(' ', get_file())
    print()

    print('From Other Module:')
    print(' ', get_caller.call_func(get_caller.get_caller_info))
    print(' ', get_caller.call_func(get_caller.get_caller_file))
    print()

if __name__ == '__main__':
    main()
