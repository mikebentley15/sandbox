# Paintful Intersections

Filbert is an intern at Lucid Software, and he is working on a bug in
Lucidpress.  Some of the shapes on the documents are missing color.  Filbert
checks the code and quickly notices where the mistake is: one of the rendering
functions is painting the intersection of shapes twice, so the browser is
running out of paint!  Help Filbert fix this function before more users'
documents lose their color.

In order to prevent all this waste, Filbert must only paint intersection of
shapes once.  He needs code that calculates the area of the intersection of a
pair of convex polygons.

A couple of examples

```
        ______________
        |            |
        |            |
        |            |
  ______|______      |
  |     |+++++|      |
  |     |+++++|      |
  |     --------------
  |           |
  |           |
  |           |
  -------------
```

If the side length is $s = 10$, then the intersection area is $s^2/4 = 10^2 / 4
= 25$.

```
________________
\      /\      /
 \    /++\    /
  \  /++++\  /
   \/++++++\/
   /\++++++/\
  /  \++++/  \
 /    \++/    \
/______\/______\
```

If the side length is $s = 10$, then the area is $s^2 \sqrt{3}/8 = 10^2
\sqrt{3}/8 = 21.651$.

Fortunately, Filbert knows the general solution for calculating the area of a
polygon.  Given vertices in order (clockwise or counterclockwise), $(x_1, y_1),
(x_2, y_2), (x_3, y_3), \ldots, (x_n, y_n)$ and treating $(x_0, y_0)$ as $(x_n,
y_n)$, the area is:

\begin{align*}
  A
   &=
    \frac{1}{2}
    \left|
     \sum_{i=1}^n
     ( x_{i-1} y_i - x_i y_{i-1} )
    \right|
  \\
   &=
    \frac{1}{2}
    \Bigg|
     (x_n y_1 - x_1 y_n)
     +
     (x_1 y_2 - x_2 y_1)
     +
     (x_2 y_3 - x_3 y_2)
     +
     \cdots
     +
     (x_{n-1} y_n - x_n y_{n-1})
    \Bigg|
\end{align*}

Some additional info:

- Membership of a point $(x, y)$ in the polygon is determined by the terms
  $(x - x_i) (y_{i-1} - y_i) - (y - y_i)(x_{i-1} - x_i)$.  If all are positive,
  it is inside a counterclockwise polygon.  If all are negative, it is inside a
  clockwise polygon.  If at least one term is zero, it is on the boundary.
  Recall that floating point computations are not exact and my requie some
  small tolerance.
- The intersection of two convex polygons is convex.
- Non-parallel lines $(x_1, y_1), (x_2, y_2)$ and $(x_3, y_3), (x_4, y_4)$
  intersect at

\begin{equation*}
 x
  =
   \frac{
    (x_1 y_2 - y_1 x_2)
    (x_3 - x_4)
    -
    (x_1 - x_2)
    (x_3 y_4 - y_3 x_4)
   }{
    (x_1 - x_2)
    (y_3 - y_4)
    -
    (y_1 - y_2)
    (x_3 - x_4)
   }
\end{equation*}

and similarly for $y$


## Input

The input describes two convex polygons.  The first line is an integer $3 \leq
N < 100$, the number of sides for the first polygon.  The next $N$ lines are
space-separated decimal coordinates of the first polygon's vertices, each
between $-1000$ and 1000 inclusive.  They are followed by an integer $3 \leq M
< 100$, the number of sides for the second polygon.  The last $M$ are the
coordinates for the second set of vertices in the same fashion.

The vertices for each polygon can be given in any order.  Hint: One way to
order vertices is computing atan2, using a selected vertex as the origin.)

Don't forget about cases where the polygons are completely disjoint (the area
is zero) or where one polygon is contained entirely within the other (the area
is the area of the contained polygon)!


## Output

The area of the intersection of the two polygons, accurate to within 0.01.


## Examples

**Input**

```
4
0.0 0.0
10.0 10.0
0.0 10.0
10.0 0.0
4
5.0 5.0
15.0 15.0
5.0 15.0
15.0 5.0
```

**Output**

```
25.00
```

**Input**

```
3
0 0
10 0
5 8.66025403784
3
0 8.66025403784
10 8.66025403784
5 0
```

**Output**

```
21.65
```


