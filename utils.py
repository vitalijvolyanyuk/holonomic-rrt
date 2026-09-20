'''
utils.py

Includes:
    - plotting helpers
'''

import numpy as np
import matplotlib.pyplot as plt


def plot(title, trees, path, path_smoothed, start, goal, obstacles, mins, maxs):
    '''
    One figure: boundary, obstacles, tree edges, raw path, smoothed path.
    '''
    plt.figure(figsize=(6, 6))

    # Boundary
    (x_min, y_min), (x_max, y_max) = mins, maxs
    plt.plot([x_min, x_max, x_max, x_min, x_min],
             [y_min, y_min, y_max, y_max, y_min], 'k-', linewidth=1.0)

    # Obstacles
    for (x0, y0), (x1, y1) in obstacles:
        plt.plot([x0, x1], [y0, y1], 'k-', linewidth=1.0)

    # Trees
    for tree in trees:
        for node in tree:
            if node.parent is not None:
                plt.plot([node.config[0], node.parent.config[0]],
                         [node.config[1], node.parent.config[1]], 'k-', linewidth=0.25)

    # Paths
    if path is not None:
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], 'r-', linewidth=1.0, label='path')

        path_smoothed = np.array(path_smoothed)
        plt.plot(path_smoothed[:, 0], path_smoothed[:, 1], 'b-', linewidth=1.5, label='smoothed')
        plt.legend(loc='lower right')

    # Start and goal
    label_style = dict(textcoords='offset points', xytext=(6, -12), zorder=4,
                       bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='none'))
    plt.plot(*start, 'ko', zorder=3)
    plt.plot(*goal, 'ko', zorder=3)
    plt.annotate('start', start, **label_style)
    plt.annotate('goal', goal, **label_style)

    plt.title(title)
    plt.axis('equal')
    plt.axis('off')
    

def show():
    plt.show()