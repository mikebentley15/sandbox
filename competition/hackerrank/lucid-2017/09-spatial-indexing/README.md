# Spatial Indexing

A lucidchart or Lucidpress page has a collection of items.  When the user
performs an action (a click or drag, for example), we often need to find nearby
items to know how to handle the user input (select a block, connect a line).

For this problem, we'll simplify each item to just its center point.  Your task
is to quickly find the item closest to where a user action occurs.

Searching through the entire list takes too long if there are lots of items.
The next section gives some information on k-d trees, whichc an be used to
create 2-d tree, for efficient spatial searching.


## k-d Tree

A $k$-dimensional tree is a space-partitioning data structure devised to allow
fast querying of points in $k$-dimensions.

The implementation here is a binary tree, where each level partitions the
points along an axis, which cycles as you descend the tree.  In two dimensions,
it would divide first by $x$, then $y$, then $x$, etc.  Each node contains a
single point, a left sub-tree of points less than or equal to it, and a right
sub-tree of points are greater than or equal to it.

The following recursive algorithm taken from Wikipedia constructs a balanced
k-d tree using a median-finding sort.

```c++
function kdtree (list of points pointList, int depth)
{
    // Select axis based on depth so that axis cycles through all valid values
    axis := depth mod k;

    // Sort pointList and choose median as pivot
    select median by axis from pointList;

    // Create node and construct subtree
    node.location := median;
    node.leftChild := kdtree(points in pointList before median, depth+1);
    node.rightChild := kdtree(points in pointList after median, depth+1);
    return node;
}
```

Once the k-d tree is constructed, the algorithm for finding the closest point
is roughly:

1. Find the closest point, among the node's point and the subtree containing
   the search location.
2. If the search location is farther from #1 than from the axis, search the
   other subtree as well.
3. Return the closest point of #1 and #2.

Reminder: Though k-d trees can work for any number of dimensions, for this
problem, $k = 2$.


## Input

The first line is an integer $0 < N \leq 2 \cdot 10^5$, the number of points on
the page.  Each of the next $N$ lines consist of the space-separated integer
coordinates between $-10^4$ and $10^4$ inclusive.

Next is a line with an integer $0 < T \leq 2 \cdot 10^5$, the number of user
actions.  Each of the next $T$ lines is an integer point, in the same format as
before.


## Output

For each of the $T$ actions, output the point on the page that is closest to
the action point.  Use the same format as the input.  If two points on the page
are the same distance away, break ties by the smallest $x$, then the smallest
$y$.


## Examples

**Input**

```
6
2 3
4 7
5 4
7 2
8 1
9 6
2
2 8
6 4
```

**Output**

```
4 7
5 4
```

