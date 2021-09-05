import os

print('importing all aliases')

cd = os.chdir
ils = os.ilistdir
ls = os.listdir
mv = os.rename
pwd = os.getcwd
rm = os.remove
rmdir = os.rmdir
uname = os.uname
mkdir = os.mkdir

def mkdir_p(path):
    try:
        mkdir(path)
    except OSError as ex:
        if ex.errno != 17:
            raise ex

def read(fname):
    with open(fname, 'r') as f:
        return f.read()

def cat(fname):
    print(read(fname))

def pp(item):
    'pretty-print'
    if isinstance(item, list):
        if len(item) < 2:
            print(repr(item))
        else:
            print('[', end='')
            first = True
            for i in item:
                if not first:
                    print(',\n ', end='')
                first = False
                print(repr(i), end='')
            print(' ]')
    elif isinstance(item, dict):
        if len(item) < 2:
            print(item)
        else:
            print('{', end='')
            first = True
            for k, v in item.items():
                if not first:
                    print(',\n ', end='')
                first = False
                print(f'{repr(k)}: {repr(v)}', end='')
            print(' }')
    else:
        print(repr(item))
