# Implimentation of RRT for point mass (Configuration: R^3)
import numpy as np
import pybullet as p

import pybullet_planning as pp
from pybullet_planning import get_delta
from pybullet_planning import INF
from util import drawPoint

from RRTStar import rrt_star

def sample_line(segment, step_size=.02):
    # generator for sampling points along a given segment
    (q1, q2) = segment
    diff = get_delta(q1, q2)
    dist = np.linalg.norm(diff)
    for l in np.arange(0., dist, step_size):
        #print("samp line")
        yield np.array(q1) + l * diff / dist
    yield q2

# RRT class
class RRT:
    def __init__(self, init_node, goal_node, config_box=((-10, -10, 0), (10, 10, 3)), max_iter=300, margin=0.5):
        # init_node: Initial node np.array(x, y, z)
        # goal_node: Goal node np.array(x, y, z)
        # config_box: Bounding box of the configuration space ((x, y, z), (x, y, z)) (since uniformly sampling in infinitely large space isn't efficient)
        # max_iter: Maximum number of expansions
        # self.nodes: list of nodes in R^3  [nodes] x [x, y, z]
        # self.paths: list of paths (start and end points between two nodes)  [paths] x [x, y, z, x, y, z]

        self.start = init_node
        self.goal = goal_node
        self.obstacles = None

        self.nodes = [init_node]
        self.goal = goal_node
        self.paths = []
        self.xrange = [config_box[0][0], config_box[1][0]]
        self.yrange = [config_box[0][1], config_box[1][1]]
        self.zrange = [config_box[0][2], config_box[1][2]]
        self.max_iter = max_iter
        self.goal_flag = False
        self.margin = margin

    # Random Sampler
    def _sampleNode(self):
        #print("SAMPLED:")
        samp = np.array([np.random.uniform(self.xrange[0],self.xrange[1]), np.random.uniform(self.yrange[0],self.yrange[1]), np.random.uniform(self.zrange[0],self.zrange[1])])
        #print(samp)
        #drawPoint(samp, [0,1,1], 0.01)
        return samp


    #     # Collision Checker for path
    # def _pathCollisionCheck(self, p1, p2):
    #     pathCollisionInfo = p.rayTest((p1,p2))
    #     if pathCollisionInfo[0] == -1:
    #         return False
    #     else:
    #         return True
        
    # Collision Checker for node
    def _pointCollisionCheck(self, point):
        #return False
        for i in range(self.obstacles.num_obstacles):
            # A B C matrices of hyperplanes, shape:(number of simplex, 3)
            ABC_matrices = self.obstacles.convexHulls[i].equations[:, 0:3]

            # D matrix of hyperplanes shape:(number of simplex, 1)
            D_matrices = self.obstacles.convexHulls[i].equations[:, -1, np.newaxis]

            # Base position of convex hull
            basePosition = self.obstacles.basePositions[i]

            # Check if point is inside convexhull
            print(point)
            print(point.shape)
            print(basePosition)
            print(basePosition.shape)
            drawPoint(basePosition, [0,0,1], 0.5)
            if np.all(np.matmul(ABC_matrices, point[:, np.newaxis]-basePosition[:, np.newaxis])+D_matrices <= 0):
                #print("collision")
                drawPoint(basePosition, [0,1,1], 1.5)
                drawPoint(point, [1,0,0], 0.5)
                return True
        #print("no collision")
        drawPoint(point, [0,1,0], 0.5)
        return False


    def _extend_func(self, p1, p2):
        for q in sample_line(segment=(p1, p2)):
            yield q

    def _distance_func(self, p1, p2):
        return np.linalg.norm(p1 - p2)

    # Find the closest neighbor
    def _findNeighbor(self, new_node):
        # Find a node that has the shortest Euclidean distance
        # NOTE: Nicer to make "node" an object
        shortest_idx = None
        shortest_dist = 100
        for idx, node in enumerate(self.nodes):
            # Compute Euclidean distance for each node
            dist = np.linalg.norm(node - new_node)
            # Update index for the closest node
            if shortest_dist > dist:
                shortest_idx = idx
        return self.nodes[shortest_idx]

    # Steering function (simple straight line so we don't actually need this func)
    def _findPath(self, new_node, neighbor_node):
        return [new_node, neighbor_node]
    
    def runAlgorithm(self, obstacles):
        self.obstacles = obstacles
        
        
        # path = pp.rrt(self.start, self.goal, self._distance_func, self._sampleNode, self._extend_func, self._pointCollisionCheck,
        #                    max_iterations=INF)
        path = rrt_star(self.start, self.goal, self._distance_func, self._sampleNode, self._extend_func, self._pointCollisionCheck,
                                radius=1,
                            max_iterations=100, max_time=20.0, verbose=True, informed=False, goal_probability=0.0000001)
        print("FOUND PATH")
        print(path)
        return path