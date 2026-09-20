'''
benchmark.py
'''

import numpy as np
from cspace import CSpace
from rrt_extend import rrt_extend
from rrt_connect import rrt_connect
from bidirectional_rrt import bidirectional_rrt

SEEDS = range(100)

PLANNERS = [('RRT-Extend', rrt_extend),
            ('RRT-Connect', rrt_connect),
            ('Bidirectional RRT-Connect', bidirectional_rrt)]


def path_length(path):
    '''
    Returns path length, or NaN if no path was found.
    '''
    if path is None:
        return np.nan
    path = np.array(path)
    return np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))


def run(planner, start, goal, cspace, seed):
    '''
    One run of a planner on one seed. 
    
    Returns its metrics.
    '''
    np.random.seed(seed)
    cspace.collision_checks = 0

    *trees, path, path_smoothed, iterations = planner(start, goal, cspace)

    return dict(success=path is not None,
                iterations=iterations,
                checks=cspace.collision_checks,
                nodes=sum(len(tree) for tree in trees),
                raw=path_length(path),
                smoothed=path_length(path_smoothed))


def benchmark(scenario, start, goal, cspace):
    '''
    Runs every planner over all seeds for one scenario and prints median metrics.
    '''
    print(f'{scenario}')

    for name, planner in PLANNERS:
        results = [run(planner, start, goal, cspace, seed) for seed in SEEDS]

        print(f'{name}')
        print(f"  success rate:      {np.mean([r['success'] for r in results]):.0%}")
        print(f"  median iterations: {np.median([r['iterations'] for r in results]):.0f}")
        print(f"  median checks:     {np.median([r['checks'] for r in results]):.0f}")
        print(f"  median nodes:      {np.median([r['nodes'] for r in results]):.0f}")
        print(f"  median raw length: {np.nanmedian([r['raw'] for r in results]):.2f}")
        print(f"  median smoothed:   {np.nanmedian([r['smoothed'] for r in results]):.2f}")

    print()


#######################################################################
# WALLS

START = [1, 1]
GOAL = [9, 9]

# Map boundaries
MINS = [0, 0]
MAXS = [10, 10]

OBSTACLES = [((2, 0), (2, 6)),
             ((4, 4), (4, 10)),
             ((6, 0), (6, 6)),
             ((8, 4), (8, 10))]

cspace = CSpace(MINS, MAXS, OBSTACLES)

benchmark('Walls', START, GOAL, cspace)

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

MINS = [0, 0]
MAXS = [10, 10]

OBSTACLES = TUNNEL

cspace = CSpace(MINS, MAXS, OBSTACLES)

benchmark('Open rooms with tunnel', START, GOAL, cspace)

#######################################################################
# START IN TUNNEL

START = [5, 5]
GOAL = [9, 1]

MINS = [0, 0]
MAXS = [10, 10]

OBSTACLES = TUNNEL

cspace = CSpace(MINS, MAXS, OBSTACLES)

benchmark('Start in tunnel', START, GOAL, cspace)

#######################################################################
# GOAL IN TUNNEL

START = [1, 1]
GOAL = [5, 5]

MINS = [0, 0]
MAXS = [10, 10]

OBSTACLES = TUNNEL

cspace = CSpace(MINS, MAXS, OBSTACLES)

benchmark('Goal in tunnel', START, GOAL, cspace)