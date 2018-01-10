# Day 0: Weighted Mean

## Objective 

In the previous challenge, we calculated a _mean_. In this challenge, we practice
calculating a weighted mean. Check out the Tutorial tab for learning materials
and an instructional video!

## Task 

Given an array, $X$, of $N$ integers and an array, $W$, representing the
respective weights of $X$'s elements, calculate and print the weighted mean of
$X$'s elements. Your answer should be rounded to a scale of $1$ decimal place
(i.e., $12.3$ format).

## Input Format

- The first line contains an integer, $N$, denoting the number of elements in
  arrays $X$ and $W$.
- The second line contains $N$ space-separated integers describing the
  respective elements of array $X$. 
- The third line contains $N$ space-separated integers describing the
  respective elements of array $W$.

## Constraints

- $5 \leq N \leq 50$
- $0 < x_i \leq 100$, where $x_i$ is the $i^{th}$ element of array $X$.
- $0 < w_i \leq 100$, where $w_i$ is the $i^{th}$ element of array $W$.

## Output Format

Print the _weighted mean_ on a new line. Your answer should be rounded to a scale
of $1$ decimal place (i.e., $12.3$ format).

## Sample Input

```
5
10 40 30 50 20
1 2 3 4 5
```

## Sample Output

```
32.0
```

## Explanation

We use the following formula to calculate the weighted mean:

\begin{equation*}
  m_w
    =
      \frac{
        \sum_{i=0}^{N-1} (x_i \cdot w_i)
      }{
        \sum_{i=0}^{N-1} w_i
      }
  \Rightarrow
  m_w
    =
      \frac{
        10 \cdot 1
        +
        40 \cdot 2
        +
        30 \cdot 3
        +
        50 \cdot 4
        +
        20 \cdot 5
      }{
        1 + 2 + 3 + 4 + 5
      }
    =
      \frac{480}{15}
    =
      32.0
\end{equation*}

And then print our result to a scale of $1$ decimal place ($32.0$) on a new line.
