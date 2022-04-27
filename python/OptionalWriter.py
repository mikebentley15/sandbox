import csv

class OptionalWriter:
    '''
    Mimics csv.writer interface but with optional filename.  If the filename is
    None, then methods do nothing.

    If the file is given, opens the file in the __enter__() and closes in the
    __exit__().

    >>> with OptionalWriter(fname) as writer:
    ...     writer.writerow(['a', 'b', 'c'])
    '''
    def __init__(self, fname=None, encoding='utf-8', **kwargs):
        '''
        Initializes the writer and opens the file.  You either need to call
        this in a 'with' context or call the close() method.

        @param fname: Optional[str]
            Name of file to write, or None if doing no-ops.
        @param encoding: str
            File encoding of fname
        @param **kwargs: passed to csv.writer if fname is not None.
        '''
        self._fname = fname
        self._writer = None
        self._fin = None
        if fname is not None:
            self._fin = open(self.fname, 'w', encoding=encoding)
            self._writer = csv.writer(self._fin, **kwargs)

    @property
    def fname(self):
        return self._fname

    @proptery
    def dialect(self):
        if self._writer is not None:
            return self._writer.dialect

    def writerow(self, row):
        if self._writer is not None:
            return self._writer.writerow(row)

    def writerows(self, rows):
        if self._writer is not None:
            return self._writer.writerows(rows)

    def close(self):
        'Close the file (if any)'
        if self._fin is not None:
            self._fin.close()
        self._fin = None
        self._writer = None

    def __enter__(self):
        'Support for "with" context manager'
        return self

    def __exit__(self, type, value, tb):
        'Close the file, for context manager support'
        self.close()

