import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

startPos = [0, 0, 2]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
sphereID = p.loadURDF("sphere_small.urdf", startPos, startOrientation)

step = 0
print(startPos)

meshScale = [0.1, 0.1, 0.1]

vertices = [[-1.000000, -1.000000, 1.000000], [1.000000, -1.000000, 1.000000],
            [1.000000, 1.000000, 1.000000], [-1.000000, 1.000000, 1.000000],
            [-1.000000, -1.000000, -1.000000], [1.000000, -1.000000, -1.000000],
            [1.000000, 1.000000, -1.000000], [-1.000000, 1.000000, -1.000000],
            [-1.000000, -1.000000, -1.000000], [-1.000000, 1.000000, -1.000000],
            [-1.000000, 1.000000, 1.000000], [-1.000000, -1.000000, 1.000000],
            [1.000000, -1.000000, -1.000000], [1.000000, 1.000000, -1.000000],
            [1.000000, 1.000000, 1.000000], [1.000000, -1.000000, 1.000000],
            [-1.000000, -1.000000, -1.000000], [-1.000000, -1.000000, 1.000000],
            [1.000000, -1.000000, 1.000000], [1.000000, -1.000000, -1.000000],
            [-1.000000, 1.000000, -1.000000], [-1.000000, 1.000000, 1.000000],
            [1.000000, 1.000000, 1.000000], [1.000000, 1.000000, -1.000000]]

normals = [[0.000000, 0.000000, 1.000000], [0.000000, 0.000000, 1.000000],
           [0.000000, 0.000000, 1.000000], [0.000000, 0.000000, 1.000000],
           [0.000000, 0.000000, -1.000000], [0.000000, 0.000000, -1.000000],
           [0.000000, 0.000000, -1.000000], [0.000000, 0.000000, -1.000000],
           [-1.000000, 0.000000, 0.000000], [-1.000000, 0.000000, 0.000000],
           [-1.000000, 0.000000, 0.000000], [-1.000000, 0.000000, 0.000000],
           [1.000000, 0.000000, 0.000000], [1.000000, 0.000000, 0.000000],
           [1.000000, 0.000000, 0.000000], [1.000000, 0.000000, 0.000000],
           [0.000000, -1.000000, 0.000000], [0.000000, -1.000000, 0.000000],
           [0.000000, -1.000000, 0.000000], [0.000000, -1.000000, 0.000000],
           [0.000000, 1.000000, 0.000000], [0.000000, 1.000000, 0.000000],
           [0.000000, 1.000000, 0.000000], [0.000000, 1.000000, 0.000000]]

indices = [
    0,
    1,
    2,
    0,
    2,
    3,  #//ground face
    6,
    5,
    4,
    7,
    6,
    4,  #//top face
    10,
    9,
    8,
    11,
    10,
    8,
    12,
    13,
    14,
    12,
    14,
    15,
    18,
    17,
    16,
    19,
    18,
    16,
    20,
    21,
    22,
    20,
    22,
    23
]

indices = [
    # top face
    0,
    1,
    2,
    0,
    2,
    3,
    #//ground face
    6,
    5,
    4,
    7,
    6,
    4,
    #//right face
    3,
    7,
    4,
    0,
    3,
    4,
    # front face
    5,
    6,
    2,
    5,
    2,
    1,
    # left side
    1,
    0,
    4,
    5,
    1,
    4,
    # back side
    5,
    3,
    6,
    7,
    6,
    2
]

# objectshape = p.createVisualShape(shapeType=p.GEOM_MESH,
#                                rgbaColor=[1, 0, 0, 1],
#                                specularColor=[0.4, .4, 0],
#                                visualFramePosition=shift,
#                                meshScale=meshScale,
#                                vertices=vertices,
#                                indices=indices,
#                                uvs=uvs,
#                                normals=normals)
objectshape = p.createVisualShape(shapeType=p.GEOM_MESH,
                               rgbaColor=[1, 0, 0, 1],
                               specularColor=[0.4, .4, 0],
                               meshScale=meshScale,
                               vertices=vertices,
                               indices=indices)

objectCollision = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                         vertices=vertices,
                                         meshScale=meshScale)

objectID = p.createMultiBody(baseVisualShapeIndex=objectshape, baseCollisionShapeIndex=objectCollision, basePosition=[0, 0, 1])

for i in range(1000):
    pos = [step, startPos[1], startPos[2]]
    p.stepSimulation()
    # p.resetBasePositionAndOrientation(sphereID, pos,
    #                                   startOrientation)
    time.sleep(1./240.)
    step += 0.01
    # spherePos, sphereOrn = p.getBasePositionAndOrientation(sphereID)
    # spherePos, sphereOrn = p.getBasePositionAndOrientation(objectID)
    # print(spherePos, sphereOrn)
    # print(p.rayTest([100, 100, 100], [100, 100, 200]))
    p.performCollisionDetection(physicsClient)

# print(p.getDynamicsInfo(sphereID, -1))
# print(p.getCollisionShapeData(sphereID,-1))
# print(p.getAABB(sphereID, -1))
# print(type(p.getAABB(sphereID, -1)))

p.disconnect()
