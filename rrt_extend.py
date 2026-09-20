'''
rrt_extend.py
'''

import numpy as np
from core import Node, nearest, extend, backtrack, shortcut_smoothing, BLOCKED


def rrt_extend(q_start, goal, cspace, step_size=0.5, goal_bias=0.05, max_iter=10000, tolerance=0.5):
    '''
    RRT-Extend

    One tree, rooted at start. Randomly samples a target configuration with goal bias. 
    Extend expansion takes one step toward the target.

    Tunable parameters:
        - goal bias: (5% - 10%) recommended
        - step_size
        - tolerance

    Returns:
        - tree, path, smoothed path, iterations
        - tree, None, None, max_iter (if no path is found)
    '''
    q_start = np.array(q_start, dtype=float)        # defining as np.array so calls can accept lists.
    goal = np.array(goal, dtype=float)
    tree = [Node(q_start)]

    for i in range(max_iter):
        if np.random.random() < goal_bias:      # sample with goal bias
            q_rand = goal
        else:
            q_rand = cspace.sample()

        nearest_node = nearest(tree, q_rand, cspace)
        new_node, status = extend(tree, nearest_node, q_rand, cspace, step_size)

        if status == BLOCKED:     # no progress, resample
            continue

        dist = cspace.distance(new_node.config, goal)
        if dist < tolerance:
            if dist == 0:       # exactly on the goal, complete
                path = backtrack(new_node)
                path_smoothed = shortcut_smoothing(path, cspace)
                return tree, path, path_smoothed, i + 1
            
            if not cspace.collision(new_node.config, goal):     # near the goal within tolerance, but must collision check 
                goal_node = Node(goal, parent=new_node)
                tree.append(goal_node)
                path = backtrack(goal_node)
                path_smoothed = shortcut_smoothing(path, cspace)
                return tree, path, path_smoothed, i + 1
        
    return tree, None, None, max_iter