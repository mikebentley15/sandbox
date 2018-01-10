#!/usr/bin/env python3
import sys
from collections import Counter
import itertools

N, _ = (int(x) for x in sys.stdin.readline().split())

problems = [(x.split()[0], int(x.split()[1]))
            for x in sys.stdin]
counts = Counter(problems)
topics = {x[0] for x in problems}
difficulties = {x[1] for x in problems}

if len(difficulties) < N or len(topics) < N:
    print(0)
    sys.exit()

diff_map = {x : {y for y in topics if (y, x) in counts} for x in range(N)}
def gen_combos(diff_map, n, chosen):
    'Returns a list of dictionaries: topic -> difficulty'
    if n >= N:
        return [chosen]
    retval = []
    for x in diff_map[n]:
        if x not in chosen:
            new_chosen = dict(chosen)
            new_chosen[x] = n
            retval.extend(gen_combos(diff_map, n+1, new_chosen))
    return retval
combos = gen_combos(diff_map, 0, dict())

num_combos = 0
for combo in combos:
    combo_counts = [counts[(t,d)] for t,d in combo.items()]
    combo_product = 1
    for count in combo_counts:
        combo_product *= count
    num_combos += combo_product

print(num_combos)

