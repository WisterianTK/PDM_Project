import numpy as np
import pybullet as p
import time
import pybullet_data
from RRT import RRT
from Obstacles import RandomObstacles
import random

# Initialize pybullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
# Add plane
planeId = p.loadURDF("plane.urdf")

# Goal and initial nodes
goal_node = np.array([6, 6, 3])
init_node = np.array([0, 0, 1])
np.random.seed(1)
# Add text GOAl in simulation
textID = p.addUserDebugText(text="GOAL", textPosition=goal_node, textColorRGB=[0, 0, 0], textSize=1)

# Generate obstacles
obstacles = RandomObstacles(num_obstacles=400, goal_position=goal_node, initial_position=init_node)
# Initialize RRT
rrt = RRT(init_node=init_node, goal_node=goal_node)

# Set flags
goal_reached = False
removed = False

for i in range(5000):
    # Run RRT
    if not goal_reached:
        goal_reached = rrt.runAlgorithm(obstacles=obstacles)
        for path in rrt.paths:
            p.addUserDebugLine(lineFromXYZ=path[0], lineToXYZ=path[1], lineColorRGB=[1, 0, 0])
    # Remove all edges and visualize the path to the goal
    else:
        print("Path found")
        print(rrt.path_to_goal)
        # Remove all edges
        if not removed:
            p.removeAllUserDebugItems()
            removed=True
        # Add Goal text again
        textID = p.addUserDebugText(text="GOAL", textPosition=goal_node, textColorRGB=[0, 0, 0], textSize=1)
        # Add the path to the goal
        for index, node in enumerate(rrt.path_to_goal[1:], start=1):
            p.addUserDebugLine(lineFromXYZ=rrt.path_to_goal[index-1], lineToXYZ=node, lineColorRGB=[0, 1, 0], lineWidth=2)

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
