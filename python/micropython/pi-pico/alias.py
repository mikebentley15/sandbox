import os as _os
import hashlib as _hl
import binascii as _ba
import micropython as _mp
import gc as _gc
import sys as _sys

print('importing all aliases')


# =========================
# direct and simple aliases
# =========================

# filesystem and system manipulation
cd    = _os.chdir
ils   = _os.ilistdir
ls    = _os.listdir
mkdir = _os.mkdir
mv    = _os.rename
pwd   = _os.getcwd
rm    = _os.remove
rmdir = _os.rmdir
uname = _os.uname

# micropython stuff
const = _mp.const
meminfo = _mp.mem_info
free  = _gc.collect


# ======================
# more complex functions
# ======================

def read(fname):
    with open(fname, 'r') as f:
        return f.read()

def cat(fname):
    print(read(fname))

def sha256(fname=None, content=None):
    if fname and contents:
        print('sha256: cannot specify both filename and contents', file=_sys.stderr)
    if fname:
        contents = read(fname)
    if contents:
        h = _hl.sha256(contents)
        return _ba.hexlify(h.digest())
    return None

def mkdir_p(path):
    try:
        mkdir(path)
    except OSError as ex:
        if ex.errno != 17:
            raise ex

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

def ppdir(item, hidden=False, sort=True):
    contents = dir(item)
    if not hidden:
        contents = [x for x in dir(item) if not x.startswith('_')]
    if sort:
        contents.sort()
    pp(contents)

# TODO: implement file grep
# TODO: implement simple file sed (i.e., search and replace of file contents)
# TODO: implement column with csv data
# TODO: add sort and unique flags for cat()
# TODO: very simple docstrings


# list of provided aliases
def aliases():
    return sorted(x for x in dir(_sys.modules[__name__])
                  if not x.startswith('_') and x != 'aliases')
