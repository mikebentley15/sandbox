import sys
import uselect

class PollStdin:
    '''
    Sets up a poll on sys.stdin to check if input is ready to read.

    Simple example:
    
        poll = PollStdin()
        poll.start()
        while True:
            new_ch = p.try_read_one()
            if new_ch == None:
                continue
            elif new_ch == 'q':
                break
            else:
                sys.stdout.write(new_ch)
                sys.stdout.flush()
        poll.stop()

    You can equivalently use this with a context manager

        with PollStdin() as poll:
            ...

    This automatically calls the start and stop.  Or equivalently

        poll = PollStdin()
        with poll:
            ...

    This is useful if you don't want to have the polling object enabled the
    whole time, but just in specific circumstances.  You can reuse that same
    poll object in separate with statements (but not nested with statements).

    '''

    def __init__(self):
        self.poll = uselect.poll()

    def start(self):
        'Registers standard in to the poll'
        self.poll.register(sys.stdin, uselect.POLLIN)

    def stop(self):
        'Unregisters the poll from standard in'
        self.poll.unregister(sys.stdin)

    def ready(self):
        '''
        Returns True if there is at least one character ready to read.  A True
        value returned from ready() ensures that read_one() will return
        immediately and that try_read_one() will return a character and not
        None.  Depending on how buffering works on standard in for the current
        implementation, it may also mean that a full line is ready to read.
        '''
        return self.poll.poll(0)

    def read_one(self):
        '''
        Blocking call to read one character.  You may want to call ready() to
        check if this will return immediately or not.

        This call will always return one character.
        '''
        return sys.stdin.read(1)

    def readline(self):
        '''
        Blocking call to read one line.  You may want to call ready() to check
        if this will (likely) return immediately or not.

        This call will always return a full line.
        '''
        return sys.stdin.readline()

    def try_read_one(self):
        'Returns one character if ready to read, else None'
        return self.read_one() if self.ready() else None

    def try_readline(self):
        '''
        If ready to read, reads a full line.  It may be possible that this
        blocks, so be mindful that one character ready to read may not
        necessarily indicate that a full line is ready to read.

        If not ready to read, returns None.
        '''
        return self.readline() if self.ready() else None

    def __enter__(self):
        '''
        Starts the poll object for a managed context (i.e., the with statement)
        '''
        self.start()
        return self

    def __exit__(self, *args):
        '''
        Stops the poll object for a managed context (i.e., the with statement)
        '''
        self.stop()
        

