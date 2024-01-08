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

goal_node = np.array([5, 5, 5])
init_node = np.array([0, 0, 2])
obstacles = RandomObstacles(5, goal_position=goal_node)
rrt = RRT(init_node=init_node, goal_node=goal_node)

print(rrt.runAlgorithm(obstacles))

for path in rrt.paths:
    p.addUserDebugLine(path[0][:],path[1][:])
    print(path)
    
for i in range(1000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
