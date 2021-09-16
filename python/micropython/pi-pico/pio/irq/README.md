# IRQ

IRQ stands for Interrupt ReQuest.  It is basically how interrupts can be
handled using interrupt handlers (also known as callbacks).

There are 8 PIO interrupts (0-7), four of which are visible to the CPU (0-3), and the other four are only visible to the state machines within the PIO's.  I think both PIO's share the same interrupts.

## Questions

The `PIO.irq()` method takes a parameter `hard=False`, which essentially tells
the micropython instance whether or not to disable interrupts and lock the heap
before calling the callback (will be done if hard is True).  If hard is False,
then instead of calling the function immediately, it gets issued to
Micropython's scheduler, which may happen some time later, but soon.

1. Do the two PIO's share the same 8 interrupts?
2. Can state machines in one PIO wait on interrupt flags from state machines in
   the other PIO?
3. It seems in Micropython, only interrupt 0 is available.  Does that mean
   interrupts 1-3 will not be able to perform a callback?
4. What arguments are passed to the callback function (if any)
5. It seems even with `micropython.schedule(func, arg)`, you would
   still need to disable interrupts so that the same interrupt cannot interrup
   you in the middle of this scheduled call, right?
6. For PIO interrupts, how do I know which state machine had the interrupt
   installed?
7. For PIO interrupts installed through the state machine, do all interrupt
   handlers get called when interrupt 0 is issued?  Or just the one associated
   to the state machine that triggered the interrupt?  By looking at the
   implementation, it looks like the latter, but I'm not sure.
8. When exactly is the flag cleared?
9. Can multiple irq callbacks be installed?  Or just one?  One for the PIO and
   one for each state machine?

## Answers

### Number 3

I'm not sure how the implementation works this way, but it seems that callbacks
installed on state machine 1 only get triggered by IRQ 1 and not 0, 2, or 3.
Likewise for state machine 2 (is only triggered by IRQ 2).

For the PIO callback, the target will specify if it will be triggered  by IRQ
0, 1, 2, or 3, or any combination of them.  By default, it will be triggered by
any of the four.

For the state machine callback, it does not seem there is any reason for what
the flags and trigger values are for the irq callbacks.  I see that it is
simply 122 and 0 respectively.  For the pio object, the flags value is 0x100
for IRQ 0, 0x200 for IRQ 1, 0x400 for IRQ 2, and 0x800 for IRQ 3.


### Number 4

It depends on how the interrupt was installed.  If you installed it through something like

```
rp2.PIO.irq()
```

then the PIO object is sent in as the argument to the function.  However, if you installed it through

```
rp2.StateMachine.irq()
```

then the StateMachine object is sent as the argument to the function.  And the
state machine objects have the `__eq__()` operator implemented.  However, this
will be called whether or not that state machine was the one who triggered the
interrupt, right?


### Number 7

Both the PIO object and the StateMachine object have an `irq()` function that
returns the `irq` object.  This `irq` object has two methods, `flags()` and
`trigger()`.  I'm not yet sure how to interpret these values.  But it seems
that it may be a way to differentiate the source of an interrupt and act
accordingly.


### Number 9

Each state machine may have one callback.  Each PIO may have its own callback
as well.

For the state machine function, I don't know what the `target` function does.
When it was set to 1, it did not trigger, which is weird.  Using the default
value seems to work...
