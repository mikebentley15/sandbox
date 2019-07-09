from distutils.core import setup, Extension
from Cython.Build import cythonize

ext = Extension(name='fib', sources=['cfib.c', 'fib.pyx'])
setup(ext_modules=cythonize(ext))
