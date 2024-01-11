import numpy as np
import pybullet as p
import time
import pybullet_data
from RRTu import RRTu, Node
from Obstacles import RandomObstacles
from util import drawPoint, drawPolynomial

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

meshScale = [.5, .5, .5]

goal_position = np.array([5, 5, 5])
init_position = np.array([0, 0, 2])
obstacles = RandomObstacles(0, goal_position=goal_position)
rrt = RRTu(init_position=init_position, goal_position=goal_position, max_iter=50)

# drawPoint([0,0,2], color=[0,1,0], size=0.2)
# drawPoint([-1,1,5], color=[1,0,0], size=0.2)


# foundneighbor, vel, acc, dt = rrt._findNeighbor(Node(np.array([-1,1,5])))

# print(foundneighbor)
# print(vel)
# print(acc)

# drawPolynomial(foundneighbor.position, foundneighbor.velocities, acc, dt)

rrt.runAlgorithm(obstacles)
print("Done")
# print(rrt.runAlgorithm(obstacles))

# for path in rrt.paths:
#     p.addUserDebugLine(path[0][:],path[1][:])
#     print(path)
    
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
