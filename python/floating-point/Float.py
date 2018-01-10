from sympy import Rational as R

class Float(object):
    def __init__(self, sign=1, significand=0, exponent=0):
        self.sign = sign
        self.significand = significand
        self.exponent = exponent

    def rat(self):
        return R(self.sign * self.significand * (2**self.exponent))

    def eval(self):
        return self.sign * self.significand * (2**self.exponent)

    def __str__(self):
        return '{} * 2^{}'.format(self.sign * self.significand, self.exponent)

    def __repr__(self):
        return 'Float: (s, sig, exp) = ({}, {}, {}) = {}'.format(
                self.sign,
                self.significand,
                self.exponent,
                str(self))

    def __lt__(self, other):
        'The less than operator "<"'
        raise NotImplementedError('Not yet implemented')

    def __le__(self, other):
        'The less than or equal to operator "<="'
        return not (self > other)

    def __eq__(self, other):
        'The equal to operator "=="'
        raise NotImplementedError('Not yet implemented')

    def __ne__(self, other):
        'The not equal to operator "!="'
        return not (self == other)

    def __gt__(self, other):
        'The greater than operator ">"'
        raise NotImplementedError('Not yet implemented')

    def __ge__(self, other):
        'The greater than or equal to operator ">="'
        return not (self < other)

    def __add__(self, other):
        raise NotImplementedError('Not yet implemented')

    def __sub__(self, other):
        raise NotImplementedError('Not yet implemented')

    def __mul__(self, other):
        raise NotImplementedError('Not yet implemented')

    def __matmul__(self, other):
        raise NotImplementedError('Not yet implemented')

    def __truediv__(self, other):
        raise NotImplementedError('Not yet implemented')

    def __floordiv__(self, other):
        raise NotImplementedError('Not yet implemented')

    def __mod__(self, other):
        raise NotImplementedError('Not yet implemented')

    def __divmod__(self, other):
        raise NotImplementedError('Not yet implemented')

    def __pow__(self, other, modulo=1):
        raise NotImplementedError('Not yet implemented')

    def __lshift__(self, other):
        raise NotImplementedError('Not yet implemented')

    def __rshift__(self, other):
        raise NotImplementedError('Not yet implemented')

    def __and__(self, other):
        raise NotImplementedError('Not yet implemented')

    def __xor__(self, other):
        raise NotImplementedError('Not yet implemented')

    def __or__(self, other):
        raise NotImplementedError('Not yet implemented')


# TODO: write unit tests

print(repr(Float(1, 10, 3)))
print(Float(1, 10, 3))
print(Float(1, 10, 3).rat())
print(Float(-1, 23, -6).rat())
