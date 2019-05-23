import a
import b
from b import myfunc as myfuncb

def myfunc():
    print('from main')

b.myfunc = myfunc

b.myfunc()
a.myfunc()
a.b.myfunc()
myfuncb()
