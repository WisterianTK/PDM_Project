import numpy as np
import pybullet as p
import time
import pybullet_data
from RRT2 import RRT
from Obstacles import RandomObstacles
from util import drawPoint


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

meshScale = [.5, .5, .5]

goal_node = np.array([5, 0, 2])
init_node = np.array([0, 0, 1])
obstacles = RandomObstacles(10, goal_position=goal_node, bounding_box=[[3, -10, 0],[4, 10, 10]])
rrt = RRT(init_node=init_node, goal_node=goal_node)

drawPoint(init_node, [1,1,1])
drawPoint(goal_node, [1,1,1])

# p.addUserDebugText("START", init_node,
#                    textColorRGB=[1, 0, 0],
#                    textSize=1.5)

# p.addUserDebugText("GOAL", goal_node,
#                    textColorRGB=[1, 0, 0],
#                    textSize=1.5)

for i in range(0,1):
    path = rrt.runAlgorithm(obstacles)
    if path != None:
        break
print(path)
if path != None:

    for i in range(len(path) - 1):
        p.addUserDebugLine(path[i],path[i+1])
        #print(path)
        
for i in range(1000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()