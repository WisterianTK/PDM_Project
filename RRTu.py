# Implimentation of RRT for point mass (Configuration: R^3)
import numpy as np
import pybullet as p

from util import drawPoint, drawPolynomial

# implementation inspired by:
# https://github.com/yijiangh/pybullet_planning/blob/dev/src/pybullet_planning/motion_planners/rrt_star.py
class Node(object):
    def __init__(self, position : np.ndarray, velocities = None, accelerations = None , parent=None, dt=0.0):
        self.position = position
        self.parent = parent
        self.dt = dt
        self.total_cost = 0.0
        self.children = set()

        
        if parent is not None:
            #self.cost = parent.cost + d
            self.parent.children.add(self)
            self.total_cost = self.parent.total_cost + dt

        if velocities is not None:
            self.velocities = velocities
        else:
            self.velocities = np.array([0.0,0.0,0.0])

        if accelerations is not None:
            self.accelerations = accelerations
        else:
            self.accelerations = np.array([0.0,0.0,0.0])

    def add_parent(self, parent, d=0):
        if parent is not None:
            self.parent = parent
            #self.cost = parent.cost + d
            self.parent.children.add(self)
    def set_dynamics(self, velocities, accelerations, dt):
        self.velocities = velocities
        self.accelerations = accelerations
        self.dt = dt
    def __str__(self):
        return 'Node : (' + str(self.position) + ')'




# RRTu class
class RRTu:
    def __init__(self, init_position, goal_position, step_size_delta_time=0.4, goal_probability=0.1, max_velocity = 5, max_acceleration = 100, config_box=((-4, -4, 0), (4, 4, 8)), max_iter=300, margin=0.5, drone_radius=0.1):
        # init_node: Initial node np.array(x, y, z)
        # goal_node: Goal node np.array(x, y, z)
        # config_box: Bounding box of the configuration space ((x, y, z), (x, y, z)) (since uniformly sampling in infinitely large space isn't efficient)
        # max_iter: Maximum number of expansions
        # self.nodes: list of nodes in R^3  [nodes] x [x, y, z]
        # self.paths: list of paths (start and end points between two nodes)  [paths] x [x, y, z, x, y, z]

        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.step_size_delta_time = step_size_delta_time
        self.drone_radius = drone_radius

        

        self.nodes = [Node(init_position)]
        self.goal = Node(goal_position)
        self.paths = []
        self.xrange = [config_box[0][0], config_box[1][0]]
        self.yrange = [config_box[0][1], config_box[1][1]]
        self.zrange = [config_box[0][2], config_box[1][2]]
        self.max_iter = max_iter
        self.goal_flag = False
        self.margin = margin
        self.goal_probability = goal_probability

        drawPoint(init_position, color=[0,1,0], size=0.2)

    def runAlgorithm(self, obstacles):
        # obstacles: class instance from Obstacles.py
        n = 0
        while n < self.max_iter:
            #print(n)
            #print(len(self.nodes))
            # Randomly sample configuration
            new_node = self._sampleNode()

            # Check if the sample configuration is collision-free
            # if self._nodeCollisionCheck(new_node, obstacles):
            #     continue

            # Find the closest neighbor
            neighbor_node, new_node_velocities, new_node_accelerations, dt = self._findNeighbor(new_node)
            if neighbor_node is None:
                #print("skip")
                continue
            
            if dt > self.step_size_delta_time:
                step_node_position, step_node_velocities = self.new_node_at_dt(neighbor_node.position, neighbor_node.velocities, new_node_accelerations)
                step_node = Node(step_node_position, step_node_velocities, new_node_accelerations, parent=neighbor_node)
            else:
                step_node = Node(new_node.position, new_node_velocities, new_node_accelerations, parent=neighbor_node)

            #drawPoint(step_node.position, color=[0,0,1], size=0.05)
            #drawPoint([-1,1,5], color=[1,0,0], size=0.2)

            drawPolynomial(neighbor_node.position, neighbor_node.velocities, new_node_accelerations, self.step_size_delta_time)



            # Find path (straight line in this simple case)
            # new_path: [ [start position] [end position] ]
            #new_path = self._findPath(new_node, neighbor_node)

            # # Check if the path is collision-free
            # if self._pathCollisionCheck(new_path):
            #     # Get new sample
            #     continue

            # Add the new node and path to the lists
            self.nodes.append(step_node)
            #self.paths.append(new_path)
            
            # Check if we found a node in the vicinity of goal node
            for node in self.nodes:
                #print(node)
                if np.linalg.norm(self.goal.position-node.position) <= self.margin:
                    self.goal_flag = True

                    # Backtrack the path to the goal

                    return self.goal_flag
            n += 1
        return self.goal_flag

    # Random Sampler
    def _sampleNode(self):
        if np.random.rand() < self.goal_probability:
            return self.goal
        position = np.array([np.random.uniform(self.xrange[0],self.xrange[1]), np.random.uniform(self.yrange[0],self.yrange[1]), np.random.uniform(self.zrange[0],self.zrange[1])])
        return Node(position)
    
    # Currently using num_segments, a step_size would be better (more consistent distances between samples)
    # However for step_size we would need the arclenght which is expensive to compute
    def sample_polynomial(start_position, start_velocity, acceleration, delta_t, num_segments=100):
        # generator for sampling points along a given polynomial
        assert len(start_position) == 3, "Input should be a numpy array of size 3"
        assert len(start_velocity) == 3, "Input should be a numpy array of size 3"
        assert len(acceleration) == 3, "Input should be a numpy array of size 3"

        # Calculate the time
        times = np.linspace(0, delta_t, num_segments+1)

        # Calculate the position
        for i, t in enumerate(times):
            yield start_position + start_velocity * t + 0.5 * acceleration * t**2

    def new_node_at_dt(self, start_position, start_velocity, acceleration):
        position = start_position + start_velocity * self.step_size_delta_time + 0.5 * acceleration * self.step_size_delta_time**2
        velocity = start_velocity + acceleration * self.step_size_delta_time
        return position, velocity

    # Collision Checker for node
    def _nodeCollisionCheck(self, new_node, obstacles):
        # False for no collision
        collision_flag = False
        eq = 0
        for i in range(obstacles.num_obstacles):
            # A B C matrices of hyperplanes, shape:(number of simplex, 3)
            ABC_matrices = obstacles.convexHulls[i].equations[:, 0:3]

            # D matrix of hyperplanes shape:(number of simplex, 1)
            D_matrices = obstacles.convexHulls[i].equations[:, -1, np.newaxis]

            # Base position of convex hull
            basePosition = obstacles.basePositions[i]

            # Check if new_node is inside convexhull
            eq = np.matmul(ABC_matrices, (new_node[:, np.newaxis]-basePosition[:, np.newaxis])/obstacles.meshScale)+D_matrices 
            condition = eq <= self.drone_radius/obstacles.meshScale
            if np.all(condition):
                return True
        #print("eq:", eq)
        return False



    # Find the closest neighbor
    def _findNeighbor(self, new_node):

        def check_dynamics(_velocities, _accelerations, _delta_t):
            #print(f"Checking: {_velocities}, {_accelerations}, {_delta_t}")
            if any(v > self.max_velocity for v in np.abs(_velocities)):
                #print("v fail")
                return False
            if any(a > self.max_acceleration for a in np.abs(_accelerations)):
                #print("a fail")
                return False
            if delta_t <= 0:
                return False
            return True
        # Find a node that has a no collision path, while adhearing to the dynamic constraints
        # cost is delta_t
        # new_node_velocities = np.empty()
        # new_node_accelerations = np.empty()
        
        candidate_dynamics = []

        for candidate in self.nodes:
            #print(candidate.position)
            delta_pos = new_node.position - candidate.position
            possible_dynamics = []

            # Max velocity case
            for sign in [-1, 1]:
                for i in range(3):  # For each dimension (x, y, z)
                    velocities = np.zeros(3)
                    accelerations = np.zeros(3)
                    # Max vel case
                    velocities[i] = sign * self.max_velocity
                    delta_t = (2*delta_pos[i])/(candidate.velocities[i] + velocities[i])
                    if delta_t > 0.0:
                        #print("dt: ", delta_t)
                        accelerations[i] = (velocities[i] - candidate.velocities[i]) / delta_t
                        for j in range(3):
                            if j != i:
                                accelerations[j] = (2/(delta_t**2))*(delta_pos[j]-candidate.velocities[j]*delta_t)
                                velocities[j] = candidate.velocities[j] + accelerations[j]*delta_t
                    

                        if not check_dynamics(velocities, accelerations, delta_t):
                            continue
                        else:
                            possible_dynamics.append((velocities, accelerations, delta_t))

            # Max acceleration case
            for sign in [-1, 1]:
                for polynomial_sign in [-1, 1]:
                    for i in range(3):
                        velocities = np.zeros(3)
                        accelerations = np.zeros(3)

                        accelerations[i] = sign * self.max_acceleration
                        delta_t = -1*candidate.velocities[i]/accelerations[i] + \
                            polynomial_sign*np.sqrt(np.square(candidate.velocities[i]/accelerations[i]) + \
                                                    2*delta_pos[i]/accelerations[i])
                        if delta_t > 0.0:
                            velocities[i] = candidate.velocities[i] + accelerations[i]*delta_t
                            for j in range(3):
                                if j != i:
                                    accelerations[j] = (2/(delta_t**2))*(delta_pos[j]-candidate.velocities[j]*delta_t)
                                    velocities[j] = candidate.velocities[j] + accelerations[j]*delta_t

                            if not check_dynamics(velocities, accelerations, delta_t):
                                continue
                            else:
                                possible_dynamics.append((velocities, accelerations, delta_t))
            
            # For every possible dynamics, check collisions
            # TODO
            
            if not possible_dynamics:
                continue
            
            min_cost = min(possible_dynamics, key=lambda x: x[2])
            candidate_dynamics.append((candidate, min_cost))

        if not candidate_dynamics:
            return None, None, None, None
        min_cost_candidate, min_cost_dyn = min(candidate_dynamics, key=lambda x: x[1][2])
        
        return min_cost_candidate, min_cost_dyn[0], min_cost_dyn[1], min_cost_dyn[2]


    # Steering function (simple straight line so we don't actually need this func)
    def _findPath(self, new_node, neighbor_node):
        return [new_node, neighbor_node]





