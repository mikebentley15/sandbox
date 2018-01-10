# Day 1: Quartiles


## Objective 

In this challenge, we practice calculating quartiles. Check out the Tutorial
tab for learning materials and an instructional video!


## Task 

Given an array, $X$, of $n$ integers, calculate the respective first quartile
($Q_1$), second quartile ($Q_2$), and third quartile ($Q_3$). It is guaranteed
that $Q_1$, $Q_2$, and $Q_3$ are integers.


## Input Format

The first line contains an integer, $n$, denoting the number of elements in the array. 

The second line contains $n$ space-separated integers describing the array's
elements.


## Constraints

- $5 \leq n \leq 50$
- $0 < x_i \leq 100$, where $x_i$ is the $i^{th}$ element of the array.


## Output Format

Print $3$ lines of output in the following order:

1. The first line should be the value of $Q_1$.
2. The second line should be the value of $Q_2$.
3. The third line should be the value of $Q_3$.


## Sample Input

```
9
3 7 8 5 12 14 21 13 18
```


## Sample Output

```
6
12
16
```

## Explanation

$X = \{3, 7, 8, 5, 12, 14, 21, 13, 18\}$. When we sort the elements in non-decreasing order, we get $X = \{3, 5, 7, 8, 12, 13, 14, 18, 21\}$. It's easy to see that $median(X) = 12$.

As there are an odd number of data points, we do not include the median (the central value in the ordered list) in either half:

```
Lower half (L): 3, 5, 7, 8
Upper half (U): 13, 14, 18, 21
```

Now, we find the quartiles:

- $Q_1$ is the $median(L)$. So, $Q_1 = \frac{5+7}{2} = 6$.
- $Q_2$ is the $median(X)$. So, $Q_2 = 12$.
- $Q_3$ is the $median(U)$. So, $Q_3 = \frac{14+18}{2} = 16$.
