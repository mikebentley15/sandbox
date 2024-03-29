# -------------------------------------------------- #
# This file is autogenerated by pioasm; do not edit! #
# -------------------------------------------------- #

import rp2
from machine import Pin
# ------------ #
# pulse_period #
# ------------ #

@rp2.asm_pio()
def pulse_period():
    set(x, 0)                             # 0
    wrap_target()
    label("1")
    set(pins, 1)                          # 1
    pull(noblock)                         # 2
    mov(x, osr)                      [4]  # 3
    mov(y, x)                             # 4
    set(pins, 0)                          # 5
    label("6")
    jmp(not_y, "1")                       # 6
    jmp(y_dec, "6")                  [8]  # 7
    wrap()


