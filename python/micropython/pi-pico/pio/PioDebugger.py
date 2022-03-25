from collections import namedtuple

DebugState = namedtuple('DebugState', ['x', 'y', 'isr', 'osr', 'tx', 'rx'])

class PioDebugger:
    '''
    A pseudo-debugger for a PIO State Machine.  It actually runs commands on
    the state machine, and is able to extract information between each executed
    statement.

    Features not available in this debugger:
    - sideset
    - merged fifos

    Properties
    ----------
    sm             state machine given in constructor
    commands       command list given in constructor
    command_index  current index in commands list (safe to write)
    is_done        (read-only) command_index >= len(commands)

    Method Summary
    --------------
    get_state()               return current state
    set_state(state)          set the state
    step()                    run current command and return (command, new state) pair
    run()                     run remaining commands and return list of states
    reset()                   clear state and set command_index to zero
    clear_state()             zeros registers and empties queues
    empty_queues()            empty fifo queues while maintaining register values
    zero_registers()          set all registers to zero
    set_register(name, val)   set a register to a 32-bit value

    Example usage:

    >>> commands = [
    ...     'set(x, 3)',
    ...     'mov(y, x)',
    ...     'mov(osr, y)',
    ...     'out(isr, 1)',
    ... ]
    >>> d = PioDebugger(rp2.StateMachine(2), commands)
    >>> d.step()
    DebugState(x=3, y=0, isr=0, osr=0, tx=[], rx=[])
    >>> d.step()
    DebugState(x=3, y=3, isr=0, osr=0, tx=[], rx=[])
    >>> d.step()
    DebugState(x=3, y=3, isr=0, osr=3, tx=[], rx=[])
    >>> d.step()
    DebugState(x=3, y=3, isr=1, osr=1, tx=[], rx=[])
    >>> d.step()
    >>> d.step()
    '''

    def __init__(self, sm, commands=None):
        '''
        Parameters:
        ----------

        sm (rp2.StateMachine)

            The state machine to use.  Should probably be stopped if you want
            to use this debugger, but not required.

        commands (list[str])

            commands to run, each as individual strings.  Does not need to
            match the program with which the state machine was initialized.
        '''
        self.sm = sm
        self.commands = commands if commands is not None else []
        self.command_index = 0

    @property
    def is_done(self):
        return command_index >= len(commands)

    def get_state(self):
        '''
        Capture the full state of the state machine safely (i.e., values remain
        the same).
        '''
        state = self._capture_state()
        self._restore_state(state)
        return state

    def set_state(self, state):
        '''Set the state'''
        # empty the tx fifo first
        while self.sm.tx_fifo():
            self.sm.exec('pull()')
        self._restore_state(state)

    def step(self):
        '''
        Performs one debug step.  Runs the one command, extracts the state from
        the state machine, then restores the state to the state machine and
        returns the DebugState object.

        Will return None if no more commands to run.
        '''
        try:
            command = self.commands[self.command_index]
        except IndexError:
            return None
        self.command_index += 1
        self.sm.exec(command)
        return command, self.get_state()

    def run(self):
        '''Run all remaining commands and return a list of states'''
        states = []
        while not self.is_done:
            _, state = self.step()
            states.append(state)
        return states

    def reset(self):
        '''Clear the state and restart from beginning of commands'''
        self.clear_state()
        self.command_index = 0

    def clear_state(self):
        '''Erase tx & rx fifos, and set registers to zero'''
        _ = self._capture_rx()
        self._erase_tx()
        self.zero_registers()

    def empty_queues(self):
        '''Empty all queues, while not changing register values'''
        state = self._capture_state()
        state.rx.clear()
        state.tx.clear()
        self._restore_state(state)

    def zero_registers(self):
        '''Set all registers to zero'''
        self.sm.exec('set(x, 0)')
        self.sm.exec('set(y, 0)')
        self.sm.exec('mov(isr, x)')
        self.sm.exec('mov(osr, x)')

    def set_register(self, name, val):
        '''Safely set a single register to a given 32-bit value'''
        assert name in ('x', 'y', 'isr', 'osr')
        if 0 <= val < 32 and name in ('x', 'y'):
            self.sm.exec(f'set({name}, {val})')
        else:
            state = self._capture_state()
            state_dict = {
                'x': state.x,
                'y': state.y,
                'isr': state.isr,
                'osr': state.osr,
                'tx': state.tx,
                'rx': state.rx,
            }
            state_dict[name] = val
            new_state = DebugState(
                x=state_dict['x'],
                y=state_dict['y'],
                isr=state_dict['isr'],
                osr=state_dict['osr'],
                tx=state_dict['tx'],
                rx=state_dict['rx'],
            )
            self._restore_state(new_state)

    def _erase_tx(self):
        '''
        Destructively erase all tx entries

        Preconditions: None
        Side effects:
        - empties tx fifo
        - overwrites osr with last tx fifo entry
        '''
        while self.sm.tx_fifo():
            self.sm.exec('pull()')

    def _capture_state(self):
        '''
        Captures the full debug state of the state machine, but destructively
        (with side effects).

        Preconditions: None
        Side effects:
        - empties rx & tx fifos
        - sets isr to zero
        - overwrites osr with last tx fifo entry
        '''
        rx = self._capture_rx()
        x, y, isr, osr = self._capture_registers()
        tx = self._capture_tx()
        return DebugState(x, y, isr, osr, tx, rx)

    def _capture_rx(self):
        '''
        Captures and returns the list of RX FIFO entries.

        Preconditions: None
        Side effects:  empties rx fifo
        '''
        rx = []
        while self.sm.rx_fifo():
            rx.append(self.sm.get())
        return rx

    def _capture_tx(self):
        '''
        Captures and returns the list of TX FIFO entries.

        Preconditions: rx fifo is empty
        Side effects:
        - overwrites osr
        - sets isr to zero
        - empties tx fifo
        '''
        assert self.sm.rx_fifo() == 0

        tx = []
        while self.sm.tx_fifo():
            self.sm.exec('pull()')
            tx.append(self._capture_register_by_name('osr'))
        return tx

    def _capture_registers(self):
        '''
        Captures and returns the values of all four registers.

        Preconditions: rx fifo is empty
        Side effects:  sets isr to zero
        '''
        isr = self._capture_register_by_name('isr')
        osr = self._capture_register_by_name('osr')
        x = self._capture_register_by_name('x')
        y = self._capture_register_by_name('y')
        return (x, y, isr, osr)

    def _capture_register_by_name(self, name):
        '''
        Captures the value of a given register by name and returns its value.

        Preconditions:
        - name is in ('x', 'y', 'isr', 'osr')
        - rx fifo is empty

        Side effects: sets isr to zero
        '''
        assert name in ('x', 'y', 'isr', 'osr')
        assert self.sm.rx_fifo() == 0

        if name != 'isr':
            self.sm.exec(f'mov(isr, {name})')
        self.sm.exec('push()')
        return self.sm.get()

    def _restore_state(self, state):
        '''
        Restores the full state back to the state machine

        Preconditions: tx fifo is empty
        Side effects:  sets tx & rx fifos, and all register values
        '''
        self._restore_rx(state.rx)
        self._restore_registers(state.x, state.y, state.isr, state.osr)
        self._restore_tx(state.tx)

    def _restore_rx(self, rx):
        '''
        Restores the state of the rx fifo

        Preconditions: tx fifo is empty
        Side effects:
        - replace current rx fifo with given values (in order)
        - overwrites osr (with last rx fifo entry)
        - sets isr to zero
        '''
        assert self.sm.tx_fifo() == 0
        assert len(rx) <= 4
        if len(rx) == 0:
            return # do nothing

        _ = self._capture_rx() # empty existing rx fifo
        for val in rx:
            self.sm.put(val)
            self.sm.exec('pull()')
            self.sm.exec('mov(isr, osr)')
            self.sm.exec('push()')

    def _restore_tx(self, tx):
        '''
        Restores the state of the tx fifo

        Preconditions: tx fifo is empty
        Side effects:  fills tx fifo with given values
        '''
        assert self.sm.tx_fifo() == 0
        for val in tx:
            self.sm.put(val)

    def _restore_registers(self, x, y, isr, osr):
        '''
        Restores the values of the registers.

        Preconditions: tx fifo is empty
        Side effects: all register values are restored
        '''
        self._restore_register_by_name('x', x)
        self._restore_register_by_name('y', y)
        self._restore_register_by_name('isr', isr)
        self._restore_register_by_name('osr', osr)

    def _restore_register_by_name(self, name, val):
        '''
        Restores the value of the given register.

        Preconditions:
        - name in ('x', 'y', 'isr', 'osr')
        - tx fifo is empty

        Side effects:
        - sets named register to value
        - sets osr to value too
        '''
        assert name in ('x', 'y', 'isr', 'osr')
        assert self.sm.tx_fifo() == 0

        self.sm.put(val)
        self.sm.exec('pull()')
        if name != 'osr':
            self.sm.exec(f'mov({name}, osr)')
