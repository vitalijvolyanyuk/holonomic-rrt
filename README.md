# Overview

Implementations of RRT-Extend, RRT-Connect, and Bidirectional RRT-Connect for a holonomic point robot. The planners work in any dimension, assuming the robot is holonomic. This demo defines the space as (x, y) with line-segment obstacles so the paths can be visualized. Extending this to a higher dimensional space would require updating the sampling, distance metric, and collision checking method.

The three planners are compared on four maps: a wall map with wide gaps, and three narrow tunnel maps where the start and goal are placed differently.

## Motivation

Planning in a higher dimensional space is hard, and the complexity of complete methods increases with dimension.

- Exact methods are complete over continuous C-space, but require computing C-space obstacles, which is expensive in high-dimensional spaces.

- Discrete-search methods, like A*, are complete over a graph and optimal (given an admissible heuristic). The branching factor (number of successors at each node) increases with dimension, making the graph expensive to search.

- Sampling methods like PRM explore the connectivity of the whole space, which is expensive when you only need one path from a start to a goal.

RRT weakens the requirements of complete and optimal to probabilistically complete and feasible.

- **Complete:** always finds a solution, or prove none exists.

- **Optimal:** always returns the lowest-cost solution.

- **Probabilistically Complete:** given a solvable problem, the probability of finding a solution goes to 1 as time goes to infinity.

- **Feasible:** The solution respects all constraints (C-space obstacles).

## Implementation

Rapidly-exploring random trees (RRT) [1] is a sampling-based planner. Rather than discretizing the configuration space, it samples random points and grows a tree from the start configuration toward each sample, incrementally exploring the space until the tree reaches the goal.

**RRT-Extend**
- Take one step toward the random sample.

**RRT-Connect**
- Continue stepping toward the random sample until it is either reached or blocked by an obstacle.

**Bidirectional RRT-Connect**
- Grow trees from both start and goal, with connect extension on both.
- Each iteration, one tree grows toward a random sample and the other grows toward that tree's new node. The trees swap roles each iteration.

## Results

**Walls**

[INSERT 3 IMAGES]

[INSERT EXPLANATION]

```
RRT-Extend
  success rate:      100%
  median iterations: 1700
  median checks:     2343
  median nodes:      1044
  median raw length: 29.32
  median smoothed:   22.17
RRT-Connect
  success rate:      100%
  median iterations: 998
  median checks:     1768
  median nodes:      714
  median raw length: 33.03
  median smoothed:   24.04
Bidirectional RRT-Connect
  success rate:      100%
  median iterations: 535
  median checks:     1531
  median nodes:      336
  median raw length: 32.66
  median smoothed:   24.79
```

**Open rooms with tunnel**

[INSERT 3 IMAGES]

[INSERT EXPLANATION]

```
RRT-Extend
  success rate:      79%
  median iterations: 2558
  median checks:     3060
  median nodes:      805
  median raw length: 15.03
  median smoothed:   13.15
RRT-Connect
  success rate:      82%
  median iterations: 2094
  median checks:     2610
  median nodes:      691
  median raw length: 15.90
  median smoothed:   13.29
Bidirectional RRT-Connect
  success rate:      87%
  median iterations: 1150
  median checks:     2046
  median nodes:      480
  median raw length: 17.83
  median smoothed:   13.53
```

**Start in tunnel**

[INSERT 3 IMAGES]

[INSERT EXPLANATION]

```
RRT-Extend
  success rate:      100%
  median iterations: 312
  median checks:     554
  median nodes:      66
  median raw length: 7.30
  median smoothed:   6.48
RRT-Connect
  success rate:      100%
  median iterations: 127
  median checks:     440
  median nodes:      62
  median raw length: 7.88
  median smoothed:   6.75
Bidirectional RRT-Connect
  success rate:      100%
  median iterations: 55
  median checks:     360
  median nodes:      50
  median raw length: 8.87
  median smoothed:   6.59
```

**Goal in tunnel**

[INSERT 3 IMAGES]

[INSERT EXPLANATION]

```
RRT-Extend
  success rate:      81%
  median iterations: 2498
  median checks:     2746
  median nodes:      753
  median raw length: 7.73
  median smoothed:   6.52
RRT-Connect
  success rate:      82%
  median iterations: 2332
  median checks:     2616
  median nodes:      742
  median raw length: 7.93
  median smoothed:   6.53
Bidirectional RRT-Connect
  success rate:      100%
  median iterations: 100
  median checks:     408
  median nodes:      64
  median raw length: 9.15
  median smoothed:   6.77
```
  

## Running the code

```bash
python3 -m venv venv
source venv/bin/activate
python3 -m pip install -r requirements.txt
```

```bash
python3 demo.py
python3 benchmark.py
```



## References

[1] S. M. LaValle, "Rapidly-Exploring Random Trees: A New Tool for Path Planning,"
Technical Report TR 98-11, Computer Science Department, Iowa State University, Oct. 1998.
