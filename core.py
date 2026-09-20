'''
core.py

Includes planner core:
    - Node
    - nearest neighbor
    - extend extension type
    - connect extension type
    - backtracking
    - shortcut smoothing
'''

import numpy as np


class Node:
    def __init__(self, config, parent=None):
        self.config = config
        self.parent = parent


def nearest(tree, q, cspace):
    '''
    Returns nearest node to configuration q in the tree.
    '''
    configs = np.array([node.config for node in tree])
    return tree[np.argmin(cspace.distances(configs, q))]


BLOCKED, ADVANCED, REACHED = 0, 1, 2


def extend(tree, node, q_target, cspace, step_size=0.5):
    '''
    Extend expansion: takes one step toward the target configuration.

    Returns the new node and a status: 
        - REACHED if it landed on the target
        - ADVANCED if it took a step
        - BLOCKED if collision
    '''
    dist = cspace.distance(node.config, q_target)

    if dist == 0:       # no step to take
        return node, REACHED

    if dist <= step_size:       # snap to target if within step size
        q_step = q_target
        status = REACHED
    else:
        v = q_target - node.config
        q_step = node.config + step_size * (v / dist)      # take a step along the direction
        status = ADVANCED

    if cspace.collision(node.config, q_step):      # collision check
        return node, BLOCKED

    new_node = Node(q_step, parent=node)
    tree.append(new_node)
    return new_node, status


def connect(tree, nearest_node, q_target, cspace, step_size=0.5):
    '''
    Connect expansion: walks toward the target configuration. (extend iterated)

    Returns the last node added and a status:
        - REACHED if it arrived at the target
        - ADVANCED if it made progress but stopped short
        - BLOCKED if nothing was added
    '''
    node = nearest_node
    status = ADVANCED

    while status == ADVANCED:
        node, status = extend(tree, node, q_target, cspace, step_size)

    if status == BLOCKED and node is not nearest_node:      # progressed before hitting an obstacle
        status = ADVANCED

    return node, status


def backtrack(node):
    '''
    Backtracks to root through parents.

    Returns path from the root to node.
    '''
    path = []
    while node is not None:
        path.append(node.config)
        node = node.parent
    return list(reversed(path))


def shortcut_smoothing(path, cspace, max_iter=1000):
    '''
    Samples two points on the path and tries to connect them with a straight segment.
    Repeats max_iter times.

    Returns path smoothed.
    '''
    for _ in range(max_iter):
        if len(path) <= 2:      # path is only 2 elements, nothing to smooth
            break

        i, j = sorted(np.random.randint(0, len(path), 2))

        if j - i < 2:       # elements are neighbors, nothing to smooth
            continue

        if not cspace.collision(path[i], path[j]):
            path = path[:i+1] + path[j:]        # if collision free, remove elements in between

    return path