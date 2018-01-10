#!/usr/bin/env python

s = reduce(lambda a,b: a+b, [bytes(chr(x)) for x in range(0x100)])

header = [hex(x)[2:] for x in range(16)]
rows = [[r'1\2'] + header]
for h2 in header:
    newrow = [h2]
    for h1 in header:
        newrow.append(chr(int(h1 + h2, base=16)))
    rows.append(newrow)

with open('tmp.txt', 'wb') as fout:
    for row in rows:
        for elem in row:
            idx = row.index(elem)
            towrite = elem
            #if row[0] in ('0', '1', '8', '9') or elem == '\x7f':
            if idx in (1, 2, 9, 10) or elem == '\x7f':
                towrite = elem.__repr__()[1:-1]
            fout.write('{0:^7}'.format(towrite))
        fout.write('\n')

with open('tmp2.txt', 'wb') as fout:
    fout.write(s)
    fout.write('\n')
