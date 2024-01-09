# Implimentation of RRT for point mass (Configuration: R^3)
import numpy as np
import pybullet as p

# RRT class
class RRT:
    def __init__(self, init_node, goal_node, config_box=((-10, -10, 0), (10, 10, 3)), max_iter=300, margin=0.5):
        # init_node: Initial node np.array(x, y, z)
        # goal_node: Goal node np.array(x, y, z)
        # config_box: Bounding box of the configuration space ((x, y, z), (x, y, z)) (since uniformly sampling in infinitely large space isn't efficient)
        # max_iter: Maximum number of expansions
        # self.nodes: list of nodes in R^3  [nodes] x [x, y, z]
        # self.paths: list of paths (start and end points between two nodes)  [paths] x [x, y, z, x, y, z]

        self.nodes = [init_node]
        self.goal = goal_node
        self.paths = []
        self.xrange = [config_box[0][0], config_box[1][0]]
        self.yrange = [config_box[0][1], config_box[1][1]]
        self.zrange = [config_box[0][2], config_box[1][2]]
        self.max_iter = max_iter
        self.goal_flag = False
        self.margin = margin

    def runAlgorithm(self, obstacles):
        # obstacles: class instance from Obstacles.py

        for i in range(self.max_iter):
            # Randomly sample configuration
            new_node = self._sampleNode()

            # Check if the sample configuration is collision-free
            if self._nodeCollisionCheck(new_node, obstacles):
                continue

            # Find the closest neighbor
            neighbor_node = self._findNeighbor(new_node)

            # Find path (straight line in this simple case)
            # new_path: [ [start position] [end position] ]
            new_path = self._findPath(new_node, neighbor_node)

            # Check if the path is collision-free
            if self._pathCollisionCheck(new_path):
                # Get new sample
                continue

            # Add the new node and path to the lists
            self.nodes.append(new_node)
            self.paths.append(new_path)

            # Check if we found a node in the vicinity of goal node
            for node in self.nodes:
                if np.linalg.norm(self.goal-node) <= self.margin:
                    self.goal_flag = True

                    # Backtrack the path to the goal

                    return self.goal_flag

        return self.goal_flag

    # Random Sampler
    def _sampleNode(self):
        return np.array([np.random.uniform(self.xrange[0],self.xrange[1]), np.random.uniform(self.yrange[0],self.yrange[1]), np.random.uniform(self.zrange[0],self.zrange[1])])

    # Collision Checker for node
    def _nodeCollisionCheck(self, new_node, obstacles):
        # False for no collision
        collision_flag = False
        for i in range(obstacles.num_obstacles):
            # A B C matrices of hyperplanes, shape:(number of simplex, 3)
            ABC_matrices = obstacles.convexHulls[i].equations[:, 0:3]

            # D matrix of hyperplanes shape:(number of simplex, 1)
            D_matrices = obstacles.convexHulls[i].equations[:, -1, np.newaxis]

            # Base position of convex hull
            basePosition = obstacles.basePositions[i]

            # Check if new_node is inside convexhull
            if np.all(np.matmul(ABC_matrices, new_node[:, np.newaxis]-basePosition[:, np.newaxis])+D_matrices <= 0):
                return True
        return False


    # Collision Checker for path
    def _pathCollisionCheck(self, new_path):
        pathCollisionInfo = p.rayTest(new_path)
        if pathCollisionInfo[0] == -1:
            return False
        else:
            return True

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