import os

import pybullet as p
import time
import pybullet_data
import scipy as sp
import numpy as np
from Obstacles import RandomObstacles
from stl import mesh


def create_convex_hull(num_vertices, xlim=(-10, 10), ylim=(-10, 10), zlim=(-10, 10)):
    # Generate random points within the specified ranges
    points = list()
    for _ in range(num_vertices):
        points += [[np.random.randint(xlim[0], xlim[1]),
                   np.random.randint(ylim[0], ylim[1]),
                   np.random.randint(zlim[0], zlim[1])]]

    points = np.array(points)
    # Compute the convex hull of the points
    convex_hull = sp.spatial.ConvexHull(points=points, qhull_options='QJ')
    return convex_hull.points


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
p.setPhysicsEngineParameter(enableSAT=True)
startPos = [0, 0, 2]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
sphereID = p.loadURDF("sphere_small.urdf", startPos, startOrientation)

step = 0

meshScale = [0.1, 0.1, 0.1]
points = create_convex_hull(6)
hull = sp.spatial.ConvexHull(np.concatenate((points, points, points, points), axis=0), qhull_options='QJ')
vertices = hull.points
indices = hull.simplices.flatten()

## Saving part
faces = hull.simplices

# Create STL mesh
stl_mesh = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
for i, face in enumerate(faces):
    for j in range(3):
        stl_mesh.vectors[i][j] = vertices[face[j]]

# Save the STL file
filename = 'temp_stl\convex_obstacle.stl'
stl_mesh.save(filename)

objectCollision = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                         fileName=filename,
                                         meshScale=meshScale)
os.remove(filename)

objectVisual = p.createVisualShape(shapeType=p.GEOM_MESH,
                                   rgbaColor=[1, 0, 0, 1],
                                   specularColor=[0.4, 0.4, 0],
                                   meshScale=meshScale,
                                   fileName=filename)

objectID = p.createMultiBody(baseVisualShapeIndex=objectVisual, baseCollisionShapeIndex=objectCollision, basePosition=[0, 0, 1])

##### TO CHECK OBSTACLE CLASS
# goal = np.array([5, 5, 5])
# num_obstacles = 50
# obstacles = RandomObstacles(num_obstacles=num_obstacles, goal_position=goal)

for i in range(1000):
    pos = [step, startPos[1], startPos[2]]
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
