from distutils.core import setup, Extension
from Cython.Build import cythonize

ext = Extension(name='sll', sources=['SinglyLinkedList.c', 'SLL.pyx'])
setup(ext_modules=cythonize(ext))
