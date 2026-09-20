'''
bidirectional_rrt.py
'''

import numpy as np
from core import Node, nearest, connect, backtrack, shortcut_smoothing, BLOCKED, REACHED


def bidirectional_rrt(q_start, goal, cspace, step_size=0.5, max_iter=10000):
    '''
    Bidirectional RRT with Connect for both trees.

    Two trees, rooted at start and goal. Each iteration one tree connects toward a
    sample, the other connects toward the first tree's new node. Trees swap roles
    each iteration so both get to explore.
    
    Tunable parameters:
        - step_size

    Returns:
        - tree_a, tree_b, path, smoothed path, iterations
        - tree_a, tree_b, None, None, max_iter (if no path is found)
    '''
    q_start = np.array(q_start, dtype=float)        # defining as np.array so calls can accept lists.
    goal = np.array(goal, dtype=float)

    tree_a = [Node(q_start)]
    tree_b = [Node(goal)]
    flipped = False 

    for i in range(max_iter):
        q_rand = cspace.sample()        # sample without goal bias since bidirectional  
        nearest_node_a = nearest(tree_a, q_rand, cspace)
        new_node_a, status_a = connect(tree_a, nearest_node_a, q_rand, cspace, step_size)

        if status_a != BLOCKED:     # tree_a made progress, try to join
            nearest_node_b = nearest(tree_b, new_node_a.config, cspace)
            new_node_b, status_b = connect(tree_b, nearest_node_b, new_node_a.config, cspace, step_size)

            if status_b == REACHED:      # trees connected
                if flipped:     # restore tree_a as the start tree
                    tree_a, tree_b = tree_b, tree_a
                    new_node_a, new_node_b = new_node_b, new_node_a

                path_start = backtrack(new_node_a)
                path_goal = backtrack(new_node_b)
                path = path_start + list(reversed(path_goal))[1:]
                path_smoothed = shortcut_smoothing(path, cspace)
                return tree_a, tree_b, path, path_smoothed, i + 1

        tree_a, tree_b = tree_b, tree_a     # swap roles every iteration
        flipped = not flipped

    #  No path found
    if flipped:     # restore tree_a as the start tree
        tree_a, tree_b = tree_b, tree_a
    return tree_a, tree_b, None, None, max_iter