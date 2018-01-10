# Proportional Resize

Each Lucidhart shape has a bounding box, modeled as the coordinates of the
top-left corner and the block's width and height.  A shape may be resized by
dragging one corner of the bounding box.  When the user resizes and holds the
shift key:

- The corner opposite the dragged one does not move.
- The hight-to-width ratio is unchanged
- At least one edge remains under the cursor.

Compute the final bounding box.

Note: This problem uses a screen coordinate system, where $x$ increases from
left to right, and $y$ increases from top to bottom.


## Input

The first line is a single integer $0 < T \leq 1000$, the number of test cases.
Each test case consists of 3 lines:

1. The bounding box as four space-separated integers, $x$, $y$, $w$, $h$, where $0 <
   x, y, w, h \leq 1000$.
2. The corner being dragged, one of `TopLeft`, `TopRight`, `BottomLeft`, or
   `BottomRight`.
3. The point the cursor's location as two space-separated integers, $a$, $b$,
   where $0 < a, b \leq 1000$.

The resize never attempts to invert the shape; e.g. when dragging the `TopLeft`
corner, the cursor remains strictly above and to the left of the stationary
`BottomRight` corner.


## Output

For each test case, output a line with the resized bounding box as four
space-separated numbers, $x$, $y$, $w$, $h$, each **rounded down** to the
nearest integer.


## Examples

**Input**

```
4
100 100 100 100
BottomRight
300 200
100 100 200 100
BottomRight
400 200
200 200 200 200
BottomRight
400 100
100 100 200 100
TopLeft
500 500
```

**Output**

```
100 100 200 200
100 100 300 150
200 100 200 100
300 200 600 300
```

