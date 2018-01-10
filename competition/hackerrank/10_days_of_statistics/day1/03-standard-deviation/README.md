# Day 1: Standard Deviation

## Objective 

In this challenge, we practice calculating standard deviation. Check out the
Tutorial tab for learning materials and an instructional video!

## Task 

Given an array, $X$, of $N$ integers, calculate and print the standard
deviation. Your answer should be in decimal form, rounded to a scale of 1
decimal place (i.e., 12.3 format). An error margin of $\pm 0.1$ will be
tolerated for the standard deviation.

## Input Format

The first line contains an integer, $N$, denoting the number of elements in the
array. 

The second line contains $N$ space-separated integers describing the respective
elements of the array.

## Constraints

- $5 \leq N \leq 100$
- $0 < x_i \leq 10^5$, where $x_i$ is the $i^{th}$ element of array $X$.

## Output Format

Print the standard deviation on a new line, rounded to a scale of 1 decimal
place (i.e., 12.3 format).

## Sample Input

```
5
10 40 30 50 20
```

## Sample Output

```
14.1
```

## Explanation

First, we find the mean: 

\begin{equation*}
  \mu
    =
      \frac{
        \sum_{i=0}^{N-1} x_i
      }{
        N
      }
    =
      30
\end{equation*}

Next, we calculate the squared distance from the mean, $(x_i - \mu)^2$, for
each $x_i$:

Now we can compute $\sum_{i=0}^{N-1} (x_i - \mu)^2 = 400 + 100 + 0 + 400 + 100
= 1000$, so:

\begin{equation*}
  \sigma
    =
      \sqrt{
        \frac{
          \sum_{i=0}^{N-1} (x_i - \mu)^2
        }{
          N
        }
      }
    =
      \sqrt{\frac{1,000}{5}}
    =
      \sqrt{200}
    =
      14.1421356
\end{equation*}

Once rounded to a scale of 1 decimal place, our result is 14.1.
