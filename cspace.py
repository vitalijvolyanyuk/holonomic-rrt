'''
cspace.py
'''

import numpy as np
from shapely.geometry import LineString


class CSpace:
    '''
    A geometric (x, y) configuration space for a point robot.

    Defines:
        - Boundary:     min, max
        - Obstacles:    line segment endpoints
        - Distance metric
        - Collision check
        - Collision check counter
    '''

    def __init__(self, mins, maxs, obstacles):
        self.mins = np.asarray(mins, dtype=float)
        self.maxs = np.asarray(maxs, dtype=float)
        self.obstacles = [LineString(seg) for seg in obstacles]
        self.collision_checks = 0

    def distance(self, q_a, q_b):
        '''
        Returns distance between configurations.

        Note: Euclidean, which assumes every coordinate is a translation.
        A revolute joint would need wrapped angular distance.
        '''
        return np.linalg.norm(q_b - q_a)

    def distances(self, configs, q):
        '''
        Returns distance from configuration q to each configuration in configs.
        '''
        return np.linalg.norm(configs - q, axis=1)

    def sample(self):
        '''
        Returns random configuration inside the boundary.
        '''
        return np.random.uniform(self.mins, self.maxs)
    
    def collision(self, q_a, q_b):
        '''
        Returns True if in collision with obstacle.
        '''
        self.collision_checks += 1

        edge = LineString([q_a, q_b])
        for seg in self.obstacles:
            if edge.intersects(seg):
                return True
        return False