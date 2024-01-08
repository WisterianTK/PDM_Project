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
goal = np.array([5, 5, 5])
num_obstacles = 100
obstacles = RandomObstacles(num_obstacles=num_obstacles, goal_position=goal)

for i in range(1000):
    #pos = [step, startPos[1], startPos[2]]
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
