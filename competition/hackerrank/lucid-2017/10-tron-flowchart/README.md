# Tron Flowchart

Through means unknown, you've been stuck in a flowchart!

One shape has an exit link.  To extricate yourself, move from shape to shape and reach the link.  There are 5 functions:

```
  down()    // move to the nearest shape directly below
  left()    // move to the nearest shape directly left
  right()   // move to the nearest shape directly right
  up()      // move to the nearest shape directly above
  random()  // randomly move to any shape (including the current)
```

What is the minimum average function calls required to reach the exit?  In
other words, what is the average cost of the optimal policy for each action
incurring a cost of one.

For example, consider the following flowchart where the exit link is at shape
$H$.

```
  A ----------- B ---- C ---- D

  |             |      |
  |      E      |      |
  |             |      |

  F ----------- G ---- H (exit)
```

- If you started at $B$, you would call `down()` and `right()`, so the fewest
  average calls are 2.
- If you started at $E$, you would first call `random()`.  There's a 1/8 chance
  you reach $B$, and call `down()` and `right()` (3 total calls).  There's 1/8
  chance you reach $H$ and exit (1 call).  There's even 1/8 chance you would
  remain at $E$, and call `random()` again (2+ calls).  After weighting each
  possible outcome by its probability, the average of the optimal strategy is
  2.667.
- If you started at $A$, you _could_ call `right()`, `down()` and `right()`,
  but it is better to call `random()` for an average of 2.667.


## Input

The positions of the shapes are given as a grid.  The first line is the number
of rows and columns $0 \leq R$, $C \leq 200$, given as space-separated
integers.  The following $R$ lines each have $C$ characters.  `^` is the
starting shape, `$` is the ending shape, `0` is another shape, and `.` is an
empty space.


## Output

The minimum average function calls, accurate to within 0.001.


## Examples

**Input**

```
3 5
0.^00
.0...
0.0$.
```

**Output**

```
2.000
```

**Input**

```
3 5
^.000
.0...
0.0$.
```

**Output**

```
2.667
```

**Input**

```
2 2
$.
.^
```

**Output**

```
2.000
```

