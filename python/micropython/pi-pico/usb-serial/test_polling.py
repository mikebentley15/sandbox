import sys
from PollStdin import PollStdin

print('Input characters: ', end='')
with PollStdin() as p:
    while True:
        new_ch = p.try_read_one()
        if new_ch == None:
            continue
        elif new_ch == 'q':
            break
        else:
            sys.stdout.write(new_ch)
            sys.stdout.flush()

