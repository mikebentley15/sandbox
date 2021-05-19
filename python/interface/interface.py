from abc import ABC, ABCMeta, abstractmethod

################################
##        abc.ABCMeta         ##
################################

class A(metaclass=ABCMeta):
    @abstractmethod
    def foo():
        'Foo description'

    @abstractmethod
    def bar():
        'Bar description'

class B(A):
    pass

assert issubclass(B, A)
try:
    B()
except TypeError:
    pass # should happen since B is also an abstract class
else:
    assert False

class C:
    def foo(self):
        pass
    def bar(self):
        pass

assert not issubclass(C, A)
assert not isinstance(C(), A)

class D(A):
    def foo(self):
        pass
    def bar(self):
        pass

assert issubclass(D, A)
assert isinstance(D(), A)

################################
##          abc.ABC           ##
################################

class E(ABC):
    @abstractmethod
    def f(self):
        'f'

    @abstractmethod
    def g(self, x):
        'g'

class F(E):
    pass

assert issubclass(F, E)
try:
    F()
except TypeError:
    pass # should happen since B is also an abstract class
else:
    assert False

class G:
    def f():
        pass
    def g():
        pass

assert not issubclass(G, E)
assert not isinstance(G(), E)

class H(E):
    def f(self):
        pass
    def g(self):
        pass

assert issubclass(H, E)
assert isinstance(H(), E)
