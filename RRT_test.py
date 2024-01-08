import numpy as np
import pybullet as p
import time
import pybullet_data
import scipy as sp
from RRT import RRT

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

meshScale = [1, 1, 1]

vertices = [np.array([-1.000000, -1.000000, 1.000000]), np.array([1.000000, -1.000000, 1.000000]),
            np.array([1.000000, 1.000000, 1.000000]), np.array([-1.000000, 1.000000, 1.000000]),
            np.array([-1.000000, -1.000000, -1.000000]), np.array([1.000000, -1.000000, -1.000000]),
            np.array([1.000000, 1.000000, -1.000000]), np.array([-1.000000, 1.000000, -1.000000])]

hull = sp.spatial.ConvexHull(points=vertices)

objectCollision = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                         vertices=hull.points,
                                         meshScale=meshScale)

objectID = p.createMultiBody(baseCollisionShapeIndex=objectCollision, basePosition=[0, 0, 1])
goal = np.array([5, 5, 5])
rrt = RRT()
for i in range(1000):
    # pos = [startPos[0], startPos[1], startPos[2]-step]
    p.stepSimulation()
    # p.resetBasePositionAndOrientation(sphereID, pos,
    #                                    startOrientation)
    time.sleep(1./240.)
    step += 0.01
    # spherePos, sphereOrn = p.getBasePositionAndOrientation(sphereID)
    # spherePos, sphereOrn = p.getBasePositionAndOrientation(objectID)
    # print(spherePos, sphereOrn)
    # print(p.rayTest([100, 100, 100], [100, 100, 200]))
    p.performCollisionDetection(physicsClient)
    print(p.getContactPoints(objectID))

# print(p.getDynamicsInfo(sphereID, -1))
# print(p.getCollisionShapeData(sphereID,-1))
# print(p.getAABB(sphereID, -1))
# print(type(p.getAABB(sphereID, -1)))

p.disconnect()
