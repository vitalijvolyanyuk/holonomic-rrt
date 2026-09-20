'''
demo.py
'''

import numpy as np
from cspace import CSpace
from rrt_extend import rrt_extend
from rrt_connect import rrt_connect
from bidirectional_rrt import bidirectional_rrt
from utils import plot, show


#######################################################################
# WALLS

START = [1, 1]
GOAL = [9, 9]
SEED = 1

# Map boundaries
MINS = [0, 0]
MAXS = [10, 10]

OBSTACLES = [((2, 0), (2, 6)),
             ((4, 4), (4, 10)),
             ((6, 0), (6, 6)),
             ((8, 4), (8, 10))]

cspace = CSpace(MINS, MAXS, OBSTACLES)

np.random.seed(SEED)
tree, path, path_smoothed, _ = rrt_extend(START, GOAL, cspace)
plot('RRT-Extend', [tree], path, path_smoothed, START, GOAL, OBSTACLES, MINS, MAXS)

np.random.seed(SEED)
tree, path, path_smoothed, _ = rrt_connect(START, GOAL, cspace)
plot('RRT-Connect', [tree], path, path_smoothed, START, GOAL, OBSTACLES, MINS, MAXS)

np.random.seed(SEED)
tree_start, tree_goal, path, path_smoothed, _ = bidirectional_rrt(START, GOAL, cspace)
plot('Bidirectional RRT-Connect', [tree_start, tree_goal], path, path_smoothed, START, GOAL, OBSTACLES, MINS, MAXS)

show()

#######################################################################

# Tunnel map shared by the tunnel scenarios
TUNNEL = [
    # bottom block
    ((3, 0),    (3, 4.95)),
    ((3, 4.95), (7, 4.95)),
    ((7, 4.95), (7, 0)),
    # top block
    ((3, 10),   (3, 5.05)),
    ((3, 5.05), (7, 5.05)),
    ((7, 5.05), (7, 10)),
]

#######################################################################
# OPEN ROOMS WITH TUNNEL

START = [1, 1]
GOAL = [9, 1]
SEED = 1

MINS = [0, 0]
MAXS = [10, 10]

OBSTACLES = TUNNEL

cspace = CSpace(MINS, MAXS, OBSTACLES)

np.random.seed(SEED)
tree, path, path_smoothed, _ = rrt_extend(START, GOAL, cspace)
plot('RRT-Extend', [tree], path, path_smoothed, START, GOAL, OBSTACLES, MINS, MAXS)

np.random.seed(SEED)
tree, path, path_smoothed, _ = rrt_connect(START, GOAL, cspace)
plot('RRT-Connect', [tree], path, path_smoothed, START, GOAL, OBSTACLES, MINS, MAXS)

np.random.seed(SEED)
tree_start, tree_goal, path, path_smoothed, _ = bidirectional_rrt(START, GOAL, cspace)
plot('Bidirectional RRT-Connect', [tree_start, tree_goal], path, path_smoothed, START, GOAL, OBSTACLES, MINS, MAXS)

show()

#######################################################################
# START IN TUNNEL

START = [5, 5]
GOAL = [9, 1]
SEED = 1

MINS = [0, 0]
MAXS = [10, 10]

OBSTACLES = TUNNEL

cspace = CSpace(MINS, MAXS, OBSTACLES)

np.random.seed(SEED)
tree, path, path_smoothed, _ = rrt_extend(START, GOAL, cspace)
plot('RRT-Extend', [tree], path, path_smoothed, START, GOAL, OBSTACLES, MINS, MAXS)

np.random.seed(SEED)
tree, path, path_smoothed, _ = rrt_connect(START, GOAL, cspace)
plot('RRT-Connect', [tree], path, path_smoothed, START, GOAL, OBSTACLES, MINS, MAXS)

np.random.seed(SEED)
tree_start, tree_goal, path, path_smoothed, _ = bidirectional_rrt(START, GOAL, cspace)
plot('Bidirectional RRT-Connect', [tree_start, tree_goal], path, path_smoothed, START, GOAL, OBSTACLES, MINS, MAXS)

show()

#######################################################################
# GOAL IN TUNNEL

START = [1, 1]
GOAL = [5, 5]
SEED = 1

MINS = [0, 0]
MAXS = [10, 10]

OBSTACLES = TUNNEL

cspace = CSpace(MINS, MAXS, OBSTACLES)

np.random.seed(SEED)
tree, path, path_smoothed, _ = rrt_extend(START, GOAL, cspace)
plot('RRT-Extend', [tree], path, path_smoothed, START, GOAL, OBSTACLES, MINS, MAXS)

np.random.seed(SEED)
tree, path, path_smoothed, _ = rrt_connect(START, GOAL, cspace)
plot('RRT-Connect', [tree], path, path_smoothed, START, GOAL, OBSTACLES, MINS, MAXS)

np.random.seed(SEED)
tree_start, tree_goal, path, path_smoothed, _ = bidirectional_rrt(START, GOAL, cspace)
plot('Bidirectional RRT-Connect', [tree_start, tree_goal], path, path_smoothed, START, GOAL, OBSTACLES, MINS, MAXS)

show()