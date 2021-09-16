import rp2

pio0 = rp2.PIO(0)
sm1 = pio0.state_machine(1)
sm2 = pio0.state_machine(2)

@rp2.asm_pio()
def irq_test():
    # sleep about 1000 cycles before each irq signal (500ms)

    set(x, 5)
    label('loop')

    set(y, 31) [31]
    label('sleep_0')
    jmp(y_dec, 'sleep_0') [31]
    irq(0)  [31]

    set(y, 31) [31]
    label('sleep_1')
    jmp(y_dec, 'sleep_1') [31]
    irq(1)  [31]

    set(y, 31) [31]
    label('sleep_2')
    jmp(y_dec, 'sleep_2') [31]
    irq(2)  [31]

    set(y, 31) [31]
    label('sleep_3')
    jmp(y_dec, 'sleep_3') [31]
    irq(3)  [31]

    set(y, 31) [31]
    label('sleep_4')
    jmp(y_dec, 'sleep_4') [31]
    irq(4)  [31]

    set(y, 31) [31]
    label('sleep_5')
    jmp(y_dec, 'sleep_5') [31]
    irq(5)  [31]

    set(y, 31) [31]
    label('sleep_6')
    jmp(y_dec, 'sleep_6') [31]
    irq(6)  [31]

    set(y, 31) [31]
    label('sleep_7')
    jmp(y_dec, 'sleep_7') [31]
    irq(7)  [31]

    pull(block)  # sleep forever


sm1.init(irq_test, freq=2000)
sm2.init(irq_test, freq=2000)

sm1.irq(lambda sm: print('sm1:', sm.irq().flags(), sm.irq().trigger()))
#sm1.irq(lambda sm: print('sm1-0:', sm.irq().flags(), sm.irq().trigger()),
#        trigger=0)
#sm1.irq(lambda sm: print('sm1-1:', sm.irq().flags(), sm.irq().trigger()),
#        trigger=1)

sm2.irq(lambda sm: print('sm2:', sm.irq().flags(), sm.irq().trigger()))
#sm2.irq(lambda sm: print('sm2-0:', sm.irq().flags(), sm.irq().trigger()),
#        trigger=0)
#sm2.irq(lambda sm: print('sm2-1:', sm.irq().flags(), sm.irq().trigger()),
#        trigger=1)

pio0.irq(lambda pio: print('pio:', pio.irq().flags(), pio.irq().trigger()))
#pio0.irq(lambda pio: print('pio-0:', pio.irq().flags(), pio.irq().trigger()),
#         trigger=rp2.PIO.IRQ_SM0)
#pio0.irq(lambda pio: print('pio-1:', pio.irq().flags(), pio.irq().trigger()),
#         trigger=rp2.PIO.IRQ_SM1)
#pio0.irq(lambda pio: print('pio-2:', pio.irq().flags(), pio.irq().trigger()),
#         trigger=rp2.PIO.IRQ_SM2)
#pio0.irq(lambda pio: print('pio-3:', pio.irq().flags(), pio.irq().trigger()),
#         trigger=rp2.PIO.IRQ_SM3)
