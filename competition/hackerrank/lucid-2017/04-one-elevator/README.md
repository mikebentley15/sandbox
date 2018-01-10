# One Elevator

Lucid just moved to a brand new office building on 10355 South Jordan Gateway.
On the first day, everyone arrives at exactly 9:00 AM.  Unfortunately, only one
elevator is working.

```
   ___________________
 3 |              |  |
   |              |  |
 2 |              |  |
   |              |  |
 1 |              |  |
   |             _|_ |
 0 |            |   || P P P P P P
   |            |___||_____________
-1 |                 |
   |_________________|
```

Each person needs to get from the ground floor to the floor where they work.
People working on the ground floor don't need the elevator.  Assume the
elevator takes 20 seconds to move between adjacent floors, and people don't
take any extra time to get on or off.

Given that the elevator carries a limited number of passengers at a time, when
is the soonest that everyone can be on their desired floor?


## Input

The first line is two space-separated integers: the elevator's capacity $0 < C
\leq 100$ and the number of people $0 \leq N \leq 10,000$.  Each of the next
$N$ lines is the floor $-5000 \leq F \leq 5000$ that person needs to reach.
The ground floor is zero.


## Output

The time portion of the soonest moment when everyone will be at their desk,
given as hh:mm:ss XM.


## Examples

**Input**

```
10 4
1
1
-1
-2
```

**Output**

```
09:01:20 AM
```

**Input**

```
4 9
1
5
2
3
3
4
9
0
9
```

**Output**

```
09:05:00 AM
```

**Input**

```
1 0
```

**Output**

```
09:00:00 AM
```

**Input**

```
1 1
-5000
```

**Output**

```
12:46:40 PM
```

