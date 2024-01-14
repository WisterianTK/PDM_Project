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

np.random.seed(1)

goal_position = np.array([10, 0, 2])
init_position = np.array([0, 0, 1])
# Generate obstacles
obstacles = RandomObstacles(num_obstacles=250, goal_position=goal_position, initial_position=init_position, meshScale=0.1)

rrt = RRTu(init_position=init_position, goal_position=goal_position, step_size_delta_time=0.4, max_iter=200, time_limit=3)

# drawPoint([0,0,2], color=[0,1,0], size=0.2)
# drawPoint([-1,1,5], color=[1,0,0], size=0.2)


# foundneighbor, vel, acc, dt = rrt._findNeighbor(Node(np.array([-1,1,5])))

# print(foundneighbor)
# print(vel)
# print(acc)

# drawPolynomial(foundneighbor.position, foundneighbor.velocities, acc, dt)

algoFlag = rrt.runAlgorithm(obstacles)
print("Done")
if algoFlag:
    path = list(rrt.tracePath())
    print(f"Path found!: {len(path)}")
    trajectory = []
    for i in range(len(path)-1):
        #drawPolynomial(path[i].position, path[i].velocities, path[i+1].accelerations, path[i+1].dt)
        for point in rrt.sample_polynomial(path[i].position, path[i].velocities, path[i+1].accelerations, path[i+1].dt):
            trajectory.append(point)
    trajectory = np.array(trajectory)
    # trajectory = rrt.generate_trajectory()
    print(trajectory)
    print(trajectory.shape)
    for i in range(trajectory.shape[0]-1):
        p.addUserDebugLine(lineFromXYZ=trajectory[i], lineToXYZ=trajectory[i+1], lineColorRGB=[1, 0, 0])
else:
    print("No path found")
# print(rrt.runAlgorithm(obstacles))

# for path in rrt.paths:
#     p.addUserDebugLine(path[0][:],path[1][:])
#     print(path)
    
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
