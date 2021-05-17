import unittest

class Base:
    pass

class A(Base):
    pass

class B(Base):
    pass

class LastOne(Base):
    pass

def all_derived_classes(base_class):
    '''
    Given the base class, find all classes in the current global scope that are
    derived from that base class.

    This returns the derived classes as a dictionary for name -> class.
    '''
    g = globals().copy()
    return {name: obj for name, obj in g.items()
            if isinstance(obj, type)
            and issubclass(obj, base_class)
            and obj != base_class}

class FoundBases(unittest.TestCase):
    def test_all_derived_classes_TestCase(self):
        classes = all_derived_classes(unittest.TestCase)
        self.assertEqual(classes, {'FoundBases': FoundBases})

    def test_all_derived_classes_Base(self):
        classes = all_derived_classes(Base)
        self.assertEqual(classes, {'A': A, 'B': B, 'LastOne': LastOne})

def main():
    unittest.main()

if __name__ == '__main__':
    main()
