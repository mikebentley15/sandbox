#!/usr/bin/env python3

from matplotlib import (
    pyplot as plt,
    animation,
    style
)
import time

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

start = time.time()

def animate(i):
    print(f'{time.time() - start}: plot #{i}')
    graph_data = open('samplefile.txt', 'r').read()
    lines = graph_data.splitlines()
    xs = []
    ys = []
    for line in lines:
        if len(line) > 1:
            x, y = line.split(',')
            xs.append(x)
            ys.append(y)

    ax1.clear()
    ax1.plot(xs, ys)

ani = animation.FuncAnimation(fig, animate, interval=33)
plt.show()
