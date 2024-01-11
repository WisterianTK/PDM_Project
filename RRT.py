# Implimentation of RRT for point mass (Configuration: R^3)
import numpy as np
import pybullet as p

# RRT class
class RRT:
    def __init__(self, init_node, goal_node, step_size=0.3, config_box=((-10, -10, 0), (10, 10, 3)), max_iter=10, margin=0.1):
        # init_node: Initial node np.array(x, y, z)
        # goal_node: Goal node np.array(x, y, z)
        # step_size: Step size for the expansion of node
        # config_box: Bounding box of the configuration space ((x, y, z), (x, y, z)) (since uniformly sampling in infinitely large space isn't efficient)
        # max_iter: Maximum number of expansions
        # self.nodes: list of nodes in R^3  [nodes] x [x, y, z]
        # self.paths: list of paths (start and end points between two nodes)  [paths] x [x, y, z, x, y, z]
        self.init_node = init_node
        self.nodes = [init_node]
        self.goal = goal_node
        self.paths = []
        self.xrange = [config_box[0][0], config_box[1][0]]
        self.yrange = [config_box[0][1], config_box[1][1]]
        self.zrange = [config_box[0][2], config_box[1][2]]
        self.max_iter = max_iter
        self.goal_flag = False
        self.margin = margin
        self.step_size = step_size
        self.path_to_goal = None

    def runAlgorithm(self, obstacles):
        # obstacles: class instance from Obstacles.py

        for i in range(self.max_iter):
            # Randomly sample configuration & get neighbor node
            new_node, neighbor_node = self._sampleNode()

            # Check if the sample configuration is collision-free
            if self._nodeCollisionCheck(new_node, obstacles):
                continue


            # Find path (straight line in this simple case)
            # new_path: [ [start position] [end position] ]
            new_path = self._findPath(neighbor_node, new_node)

            # Check if the path is collision-free
            if self._pathCollisionCheck(new_path):
                # Get new sample
                continue

            # Add the new node and path to the lists
            self.nodes.append(new_node)
            self.paths.append(new_path)

            # Check if new node is in the vicinity of goal node
            if np.linalg.norm(self.goal-new_node) <= self.margin:
                self.goal_flag = True
                # Trace back the path to the goal
                self.path_to_goal = self._tracePath(new_node=new_node)
                return self.goal_flag
        return self.goal_flag


    # Random Sampler
    def _sampleNode(self):
        # With probability of 99%, a point is sampled randomly
        # With probability of 1%, this function returns the goal node
        # This modification is mentioned on page 235 in Book "Planning Algorithm" by Steven M. Lavalle
        if np.random.rand() > 0.99:
            return self.goal, self._findNeighbor(self.goal)
        else:
            # First sample a point in configuration space
            point = np.array([np.random.uniform(self.xrange[0],self.xrange[1]), np.random.uniform(self.yrange[0],self.yrange[1]), np.random.uniform(self.zrange[0],self.zrange[1])])
            # Find the closest node to the sampled point
            neighbor_node = self._findNeighbor(point)
            # Find unit vector from the closest node to the sampled point
            unitVector = (point-neighbor_node)/np.linalg.norm(neighbor_node-point)
            # Return a point that is translated in the direction of the unit vector by the step size & neighbor node
            return neighbor_node+self.step_size*unitVector, neighbor_node

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
            condition = np.matmul(ABC_matrices, (new_node[:, np.newaxis]-basePosition[:, np.newaxis])/obstacles.meshScale)+D_matrices <= 0
            if np.all(condition):
                return True
        return False


    # Collision Checker for path
    def _pathCollisionCheck(self, new_path):
        pathCollisionInfo = p.rayTest(new_path[0], new_path[1])
        if pathCollisionInfo[0][0] == -1:
            return False
        else:
            return True

    # Find the closest neighbor
    def _findNeighbor(self, new_node):
        # # Find a node that has the shortest Euclidean distance
        # # NOTE: Nicer to make "node" an object

        # # Slow implementation
        # shortest_idx = None
        # shortest_dist = 100
        # for idx, node in enumerate(self.nodes):
        #     # Compute Euclidean distance for each node
        #     dist = np.linalg.norm(node - new_node)
        #     # Update index for the closest node
        #     if shortest_dist > dist:
        #         shortest_idx = idx
        #         shortest_dist = dist
        # return self.nodes[shortest_idx]

        # Faster implementation
        nodes = np.asarray(self.nodes)
        dist = np.linalg.norm(nodes - new_node, axis=1)
        index = np.argmin(dist)
        return self.nodes[index]

    # Steering function (simple straight line so we don't actually need this func)
    def _findPath(self, neighbor_node, new_node):
        return [neighbor_node, new_node]

    def _tracePath(self, new_node):

        # Starting from the node closest to the goal
        sorted_path = [new_node]

        # Run while loop until the sorted path has the start node at the beginning
        while not np.all(self.init_node == sorted_path[0]):
            for path in reversed(self.paths): # Reversed order is used because recently added path is more at the end due to the incremental manner
                # Find the path that leads to the first node of the sorted path
                if np.all(path[1] == sorted_path[0]):
                    # insert the node to the sorted path
                    sorted_path.insert(0,path[0])
        return sorted_path







