# Bowling Scoring

Lucid is renting out the bowling alley for an engineering activity, and we need
to verify that the scoring system is correct.

A game's score is the sum of scores of ten frames.  At the beginning of each
frame, 10 pins are set up.  Then the bowler will make up to two attempts to
knock down the pins with the ball.  If the player knocks down all 10 pins on
the first ball, there is not a second

In general, the score for a frame is the total number of pins knocked down
during the frame.  If a player bowls over 3 pins with the first show, then 6
with the second, the player would receive a total of 9 points for that frame.

In the even that all ten pins are knocked down, bonuses are awarded in addition
to the usual ten points:

- A **strike** is knocking down all ten pins with the first ball of a frame.
  The number of pins knocked down with the next two balls is added to this
  frame's score.
- A **spare** is knocking down all remaining pins with the second ball of a
  frame.  The number of pins knocked down with the next ball is added to this
  frame's score.

If the 10th frame has a strike or spare, the player gets one or two extra balls
to resolve the pending bonuses.  These extra balls are counted only for
calculating bonuses acquired on regular throws; they themselves do not make a
normal contribution, nor add additional bonuses.


## Input

The first line of the input is an integer $1 \leq N \leq 30$.

Each of the next $N$ lines is a full game represented by the number of pins
knocked down on each throw, each number separated by a space.  Every game will
be well formed; i.e. there will be exactly the correct number of throws for
each game.


## Output

For each game print out the final score each on its own line.


## Examples

**Input**

```
3
10 10 10 10 10 10 10 10 10 10 10 10
5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5 5
10 0 0 10 0 0 10 0 0 10 0 0 10 0 0
```

**Output**

```
300
150
50
```

**Input**

```
3
4 5 5 4 6 3 2 7 7 2 3 6 6 3 8 1 1 8 9 0
6 2 5 3 3 7 8 1 10 4 6 8 1 7 3 10 4 6 6
10 0 10 3 3 10 9 1 6 3 3 5 9 1 4 4 10 6 4
```

**Output**

```
90
146
137
```
