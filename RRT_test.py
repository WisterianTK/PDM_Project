import numpy as np
import pybullet as p
import time
import pybullet_data
from RRT import RRT
from Obstacles import RandomObstacles

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

meshScale = [.5, .5, .5]

goal_node = np.array([3, 3, 2])
init_node = np.array([0, 0, 2])
textID = p.addUserDebugText(text="GOAL", textPosition=goal_node, textColorRGB=[0, 0, 0], textSize=1)

obstacles = RandomObstacles(num_obstacles=100, goal_position=goal_node)
rrt = RRT(init_node=init_node, goal_node=goal_node)

goal_reached = rrt.runAlgorithm(obstacles)
print(goal_reached)

for path in rrt.paths:
    p.addUserDebugLine(lineFromXYZ=path[0], lineToXYZ=path[1], lineColorRGB=[1, 0, 0])
    # print(path)

for i in range(5000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
