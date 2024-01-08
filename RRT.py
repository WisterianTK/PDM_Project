# Implimentation of RRT for point mass (Configuration: R^3)
import numpy as np
import pybullet as p
from scipy.spatial import ConvexHull

# RRT class
class RRT:
    def __init__(self, init_node, goal_node, config_box, max_iter, object_info):
        # init_node: Initial node [x, y, z]
        # goal_node: Goal node [x, y, z]
        # bounding box of the configuration space [x, y, z, x, y, z] (since uniformly sampling in infinitely large space isn't efficient)
        # max_iter: Maximum number of expansions
        # object_info: A list which contains instances of objects
        # self.nodes: list of nodes in R^3  [nodes] x [x, y, z]
        # self.paths: list of paths (start and end points between two nodes)  [paths] x [x, y, z, x, y, z]

        self.nodes = init_node
        self.goal = goal_node
        self.paths = []
        self.xrange = [config_box[0], config_box[3]]
        self.yrange = [config_box[1], config_box[4]]
        self.zrange = [config_box[2], config_box[5]]
        self.max_iter = max_iter
        self.object_info = object_info
        self.goal_flag = "false"

    def runAlgorithm(self):
        for i in range(self.max_iter):
            # Randomly sample configuration
            new_node = self._sampleNode()

            # Check if the sample configuration is collision-free
            if self._nodeCollisionCheck(new_node) == "true":
                continue

            # Find the closest neighbor
            neighbor_node = self._findNeighbor(new_node)

            # Find path (straight line in this simple case)
            # new_path: [ [start position] [end position] ]
            new_path = self._findPath(new_node, neighbor_node)

            # Check if the path is collision-free
            if self._pathCollisionCheck(new_path) == "true":
                continue

            # Add the new node and path to the lists
            self.nodes += new_node
            self.paths += new_path

            # Check if the goal has been reached
            if self.goal in self.nodes:
                self.goal_flag = "true"
                break

        return self.nodes, self.paths

    # Random Sampler
    def _sampleNode(self):
        return [np.random.uniform(self.xrange), np.random.uniform(self.yrange), np.random.uniform(self.zrange)]

    # Collision Checker for node
    def _nodeCollisionCheck(self, new_node):
        collision_flag = "false"
        for i in self.object_info:
            box_min = i[0]
            box_max = i[1]
            # Check if new_node is inside bounding boxes
            if box_min <= new_node <= box_max:
                collision_flag = "true"
                break
        return collision_flag


    # Collision Checker for path
    def _pathCollisionCheck(self, new_path):
        pathCollisionInfo = p.rayTest(new_path)
        if pathCollisionInfo[0] == -1:
            return "false"
        else:
            return "true"

    # Find the closest neighbor
    def _findNeighbor(self, new_node):
        # Find a node that has the shortest Euclidean distance
        # NOTE: Nicer to make "node" an object
        shortest_idx = None
        shortest_dist = 0
        for node, idx in enumerate(self.nodes):
            # Compute Euclidean distance for each node
            dist = np.linalg.norm(node - new_node)
            # Update index for the closest node
            if shortest_dist > dist:
                shortest_idx = idx
        return self.nodes[shortest_idx]


    # Steering function (simple straight line so we don't actually need this func)
    def _findPath(self, new_node, neighbor_node):
        return [new_node, neighbor_node]





