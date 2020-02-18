'Test utilities'

from contextlib import contextmanager
from pathlib import Path
import importlib.util
import os
import shutil
import sys
import tempfile

def touch(filename):
    '''
    Create an emtpy file if it does not exist, otherwise updates the
    modification time.
    '''
    Path(filename).touch()

@contextmanager
def pushd(directory):
    '''
    Changes to a given directory using a with statement.  At the end of the
    with statement, changes back to the previous directory.

    >>> original_dir = os.path.abspath(os.curdir)
    >>> with tempdir() as new_dir:
    ...     temporary_directory = new_dir
    ...     with pushd(new_dir):
    ...         pushed_dir = os.path.abspath(os.curdir)
    ...     popped_dir = os.path.abspath(os.curdir)

    >>> temporary_directory == pushed_dir
    True
    >>> original_dir == popped_dir
    True
    >>> temporary_directory == popped_dir
    False
    '''
    original_dir = os.path.abspath(os.curdir)
    try:
        os.chdir(directory)
        yield
    finally:
        os.chdir(original_dir)

@contextmanager
def tempdir(*args, **kwargs):
    '''
    Creates a temporary directory using tempfile.mkdtemp().  All arguments are
    passed there.  This function is to be used in a with statement.  At the end
    of the with statement, the temporary directory will be deleted with
    everything in it.

    Test that the temporary directory exists during the block and is removed
    after
    >>> import os
    >>> temporary_directory = None
    >>> with tempdir() as new_dir:
    ...     temporary_directory = new_dir
    ...     print(os.path.isdir(temporary_directory))
    ...
    True
    >>> os.path.isdir(temporary_directory)
    False
    >>> os.path.exists(temporary_directory)
    False

    Test that an exception is not thrown if it was already deleted
    >>> import shutil
    >>> with tempdir() as new_dir:
    ...     shutil.rmtree(new_dir)

    Test that the directory is still deleted even if an exception is thrown
    within the with statement.
    >>> try:
    ...     with tempdir() as new_dir:
    ...         temporary_directory = new_dir
    ...         raise RuntimeError()
    ... except RuntimeError:
    ...     pass
    >>> os.path.isdir(temporary_directory)
    False
    '''
    new_dir = tempfile.mkdtemp(*args, **kwargs)
    try:
        yield new_dir
    finally:
        try:
            shutil.rmtree(new_dir)
        except FileNotFoundError:
            pass

def load_module_from_string(name, contents):
    'Load and return a python module from a string'
    spec = importlib.util.spec_from_loader(name, loader=None)
    module = importlib.util.module_from_spec(spec)
    exec(contents, module.__dict__)
    sys.modules[name] = module
    return module

def load_module_from_file(name, filepath):
    'Load and return a python module from a filepath'
    spec = importlib.util.spec_from_file_location(name, filepath)
    print(spec)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sys.modules[name] = module
    return module
