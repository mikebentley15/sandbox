'Provides the PPrintable class'

import pprint as _pprint

class PPrintable:
    '''
    You can inherit from this class to be able to pretty-print your class using pprint.

    >>> import pprint
    >>> class MyClass(PPrintable):
    ...     def __init__(self):
    ...         self.a = 'abcd'
    ...         self.b = None
    >>> print(MyClass())
    MyClass{'a': 'abcd', 'b': None}

    >>> c = MyClass()
    >>> c.b = 'abcdefghijklmnopqrstuvwxyz--abcdefghijklmnopqrstuvwxyz'
    >>> print(c)
    MyClass{
     'a': 'abcd',
     'b': 'abcdefghijklmnopqrstuvwxyz--abcdefghijklmnopqrstuvwxyz'}

    >>> c.d = 'hello there'
    >>> print(c)
    MyClass{
     'a': 'abcd',
     'b': 'abcdefghijklmnopqrstuvwxyz--abcdefghijklmnopqrstuvwxyz',
     'd': 'hello there'}

    >>> class OtherClass(MyClass):
    ...     def _get_kwargs(self):
    ...         return [('a', self.a)]
    >>> print(OtherClass())
    OtherClass{'a': 'abcd'}

    >>> o = OtherClass()
    >>> o.a = 'abcdefghijklmnopqrstuvwxyz--abcdefghijklmnopqrstuvwxyz--abcdef'
    >>> o.b = 123
    >>> o.d = 'hello there'
    >>> print(o)
    OtherClass{
     'a': 'abcdefghijklmnopqrstuvwxyz--abcdefghijklmnopqrstuvwxyz--abcdef'}
    '''
    def pp(self):
        _pprint.pprint(self)

    def __str__(self):
        return _pprint.pformat(self)

    def __repr__(self):
        return self.__class__.__name__ + repr(dict(self._get_kwargs()))

    def _get_kwargs(self):
        return sorted(self.__dict__.items())

def _pprint_pprintable(printer, obj, stream, indent, allowance, context, level):
    args = obj._get_kwargs()
    write = stream.write
    write(obj.__class__.__name__)
    write('{')
    if args:
        write('\n' + (indent + printer._indent_per_level) * ' ')
        printer._format_dict_items(args, stream, indent, allowance+1, context,
                                   level)
    write('}')

_pprint.PrettyPrinter._dispatch[PPrintable.__repr__] = _pprint_pprintable
