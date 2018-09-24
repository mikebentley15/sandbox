#!/usr/bin/env python3
'''
Downloads, compiles, and installs gcc or llvm.
'''

from contextlib import contextmanager
from urllib.request import urlopen
from urllib.request import urlretrieve
import argparse
import shutil
import subprocess as subp
import sys
import tarfile
import tempfile
import os

def parse_args(arguments):
    'Returned parsed arguments'
    parser = argparse.ArgumentParser()
    parser.add_argument('compiler', choices=('gcc', 'llvm'),
                        help='Compiler name, from (gcc, llvm)')
    parser.add_argument('version', help='Full version number (e.g. 5.4.0)')
    args = parser.parse_args(arguments)
    return args

def empty_and_create_dir(dirpath):
    '''
    Makes sure dirpath is an empty directory.  Will delete the directory
    contents if there, and will create the directory if it is not there.

    Similar to calling
      rm -rf <dirpath>
      mkdir -p <dirpath>
    '''
    try:
        shutil.rmtree(dirpath)
    except FileNotFoundError:
        pass
    os.makedirs(dirpath)

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
    >>> print(os.path.isdir(temporary_directory))
    False
    >>> print(os.path.exists(temporary_directory))
    False

    Test that an exception is not thrown if it was already deleted
    >>> import shutil
    >>> with tempdir() as new_dir:
    ...     shutil.rmtree(new_dir)
    '''
    new_dir = tempfile.mkdtemp(*args, **kwargs)
    try:
        yield new_dir
    finally:
        try:
            shutil.rmtree(new_dir)
        except FileNotFoundError:
            pass

def extract_tar_url(url, compression='*'):
    '''
    Download the url tar file and extract it all.  Compression can be one of
    the following:
    - '*': transparent compression (default)
    - 'gz': gzip compression
    - 'bz2': bzip2 compression
    - 'xz': xz compression
    '''
    valid_compressions = ('*', 'gz', 'bz2', 'xz')
    assert compression in valid_compressions
    if compression == 'xz':
        fname, _ = urlretrieve(url)
        with tarfile.open(fname, mode='r|xz') as tarobj:
            tarobj.extractall()
        os.remove(fname)
    else:
        with urlopen(url) as tarin:
            with tarfile.open(fileobj=tarin, mode='r|' + compression) as tarobj:
                tarobj.extractall()

def install_gcc(version, prefix='/usr/local'):
    '''
    Download, compile, and install gcc

    Will install gcc into
      prefix + '/gcc-' + version

    @param version: full version string (e.g. '5.4.0')
    @param prefix: install prefix (default '/usr/local')
    '''
    orig_dir = os.path.realpath(os.curdir)
    with tempdir(prefix='gcc-build') as gccdir:
        src = os.path.join(gccdir, 'src')
        gcc_src = os.path.join(src, 'gcc-{}'.format(version))
        build = os.path.join(gccdir, 'build', 'gcc-{}-build'.format(version))
        install = os.path.join(prefix, 'gcc-{}'.format(version))
        os.makedirs(src)
        os.makedirs(build)
        url = 'https://bigsearcher.com/mirrors/gcc/releases/' \
              'gcc-{version}/gcc-{version}.tar.gz'.format(version=version)
        os.chdir(src)
        print('Downloading gcc {} from bigsearcher.com'.format(version))
        extract_tar_url(url, compression='gz')
        os.chdir(build)
        subp.check_call([
            os.path.join(gcc_src, 'configure'),
            '--prefix={}'.format(install),
            '--disable-multilib',
            '--enable-languages=c,c++',
            ])
        subp.check_call(['make'])
        subp.check_call(['make','install'])
        os.chdir(orig_dir)

def install_llvm(version, prefix='/usr/local'):
    '''
    Download, compile, and install llvm

    Will install gcc into
      prefix + '/llvm-' + version

    @param version: full version string (e.g. '6.0.1')
    @param prefix: install prefix (default '/usr/local')
    '''
    orig_dir = os.path.realpath(os.curdir)
    with tempdir(prefix='llvm-build') as llvmdir:
        src = os.path.join(llvmdir, 'src')
        llvm_src = os.path.join(src, 'llvm-{}.src'.format(version))
        build = os.path.join(llvmdir, 'build', 'llvm-{}-build'.format(version))
        install = os.path.join(prefix, 'llvm-{}'.format(version))
        os.makedirs(src)
        os.makedirs(build)
        os.chdir(src)
        urlbase = 'http://releases.llvm.org/{}/'.format(version)
        def extract_xz(filename):
            'Helper to download and print message'
            print('Download & extract', filename)
            sys.stdout.flush()
            extract_tar_url(urlbase + filename, compression='xz')
        extract_xz('llvm-{}.src.tar.xz'.format(version))
        extract_xz('cfe-{}.src.tar.xz'.format(version))
        extract_xz('compiler-rt-{}.src.tar.xz'.format(version))
        extract_xz('polly-{}.src.tar.xz'.format(version))
        extract_xz('libcxx-{}.src.tar.xz'.format(version))
        os.rename('cfe-{}.src'.format(version),
                  os.path.join(llvm_src, 'tools', 'clang'))
        os.rename('compiler-rt-{}.src'.format(version),
                  os.path.join(llvm_src, 'projects', 'compiler-rt'))
        os.rename('polly-{}.src'.format(version),
                  os.path.join(llvm_src, 'tools', 'polly'))
        os.rename('libcxx-{}.src'.format(version),
                  os.path.join(llvm_src, 'projects', 'libcxx'))
        os.chdir(build)
        subp.check_call([
            'cmake', llvm_src,
            '-DCMAKE_INSTALL_PREFIX={}'.format(install),
            '-DLLVM_TARGETS_TO_BUILD=host',
            '-DCMAKE_BUILD_TYPE=Release',
            ])
        subp.check_call(['make'])
        subp.check_call(['make', 'install'])
        os.chdir(orig_dir)

def main(arguments):
    'Main logic here'
    args = parse_args(arguments)
    prefix = os.path.join(os.environ['HOME'], 'install')
    if args.compiler == 'gcc':
        install_gcc(args.version, prefix=prefix)
    elif args.compiler == 'llvm':
        install_llvm(args.version, prefix=prefix)
    else:
        raise ValueError('Unrecognized compiler to compile: {}'
                         .format(args.compiler))

if __name__ == '__main__':
    main(sys.argv[1:])
