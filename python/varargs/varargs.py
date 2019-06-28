

def func(*args, **kwargs):
    print(args)

def my_decorator(f):
    def helper(*args, **kwargs):
        from pprint import pprint as pp
        pp(args)
        pp(kwargs)
        return f(*args, **kwargs)
    return helper

@my_decorator
def func2(a='a', b='b'):
    print('a:', a)
    print('b:', b)
    return a + b

func('a', b='b')
func2('a', 'b')
func2('a', b='b')
