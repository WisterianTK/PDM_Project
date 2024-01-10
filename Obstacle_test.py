import pybullet as p
import time
import pybullet_data
import scipy as sp
import numpy as np
from Obstacles import RandomObstacles


# def create_convex_hull(num_vertices, xlim=(-10, 10), ylim=(-10, 10), zlim=(-10, 10)):
#     # Generate random points within the specified ranges
#     points = list()
#     for _ in range(num_vertices):
#         points += [[np.random.randint(xlim[0], xlim[1]),
#                    np.random.randint(ylim[0], ylim[1]),
#                    np.random.randint(zlim[0], zlim[1])]]
#
#     points = np.array(points)
#     # Compute the convex hull of the points
#     convex_hull = sp.spatial.ConvexHull(points=points, qhull_options='QJ')
#     return convex_hull.points


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")

startPos = [0, 0, 2]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
sphereID = p.loadURDF("sphere_small.urdf", startPos, startOrientation)


#
# meshScale = [0.1, 0.1, 0.1]
# points = create_convex_hull(8)
# hull = sp.spatial.ConvexHull(np.concatenate((points, points, points), axis=0), qhull_options='Qt')
# vertices = hull.points
#
# indices = hull.simplices.flatten()
# objectVisual = p.createVisualShape(shapeType=p.GEOM_MESH,
#                                    rgbaColor=[1, 0, 0, 1],
#                                    specularColor=[0.4, 0.4, 0],
#                                    meshScale=meshScale,
#                                    vertices=vertices,
#                                    indices=indices)
#
#
# objectCollision = p.createCollisionShape(shapeType=p.GEOM_MESH,
#                                          vertices=vertices,
#                                          meshScale=meshScale)
#
# objectID = p.createMultiBody(baseCollisionShapeIndex=objectCollision, basePosition=[0, 0, 1])

#### TO CHECK OBSTACLE CLASS
goal_node = np.array([6, 6, 3])
init_node = np.array([0, 0, 1])
np.random.seed(1)
# Add text GOAl in simulation
textID = p.addUserDebugText(text="GOAL", textPosition=goal_node, textColorRGB=[0, 0, 0], textSize=1)

# Generate obstacles
obstacles = RandomObstacles(num_obstacles=400, goal_position=goal_node, initial_position=init_node)

for i in range(1000):
    #pos = [step, startPos[1], startPos[2]]
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
