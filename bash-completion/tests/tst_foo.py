from unittest import TestCase
import os

from completion import get_completion
import testutil as util

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))

class FooCompletionTest(TestCase):
    FOO_PROG = 'foo'
    FOO_BASH_COMPLETION = os.path.join(SCRIPT_DIR, '..', 'completions', 'foo')

    def assertEqualCompletion(self, args, expected_completions):
        'Asserts that the expected completions are obtained'
        actual = get_completion(self.FOO_BASH_COMPLETION, self.FOO_PROG, args)
        self.assertEqual(sorted(expected_completions), sorted(actual))

    def test_noargs(self):
        self.assertEqualCompletion(
            '', ['--help', '--version', '--verbose', 'file', 'hostname'])

    def test_dash(self):
        self.assertEqualCompletion('-', ['--help', '--version', '--verbose'])

    def test_dash_v(self):
        self.assertEqualCompletion('--v', ['--version', '--verbose'])

    def test_file_incomplete(self):
        self.assertEqualCompletion('fi', ['file'])

    def test_hostname_incomplete(self):
        self.assertEqualCompletion('ho', ['hostname'])

    def test_file_completed(self):
        fnames = ('file1.txt', 'file2.txt', 'file3.dat')
        with util.tempdir() as new_dir:
            for fname in fnames:
                util.touch(os.path.join(new_dir, fname))
            self.assertEqualCompletion(
                'file ' + new_dir + '/',
                [os.path.join(new_dir, x) for x in fnames])
            with util.pushd(new_dir):
                self.assertEqualCompletion(
                    'file ',
                    ['--help', '--version', '--verbose'] + list(fnames))
                self.assertEqualCompletion('file file2', ['file2.txt'])
