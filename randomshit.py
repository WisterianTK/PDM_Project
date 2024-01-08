import pybullet as p
import numpy as np
import time


def generate_random_polygon(num_vertices=10, scale=1.0):
    # Generate random 3D vertices
    vertices = scale * np.random.randint(1, 5, [num_vertices, 3])

    # Create faces for a simple triangulation
    faces = []
    for i in range(1, num_vertices - 1):
        faces.append([0, i, i + 1])

    return vertices, faces

def create_pybullet_polygon(vertices, faces):
    # Create a PyBullet simulation
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -10)

    # Create a base plane for visualization
    planeId = p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, planeId)

    # Create a convex hull collision shape using the generated vertices and faces
    collision_shape_id = p.createCollisionShape(p.GEOM_MESH, vertices=vertices, meshScale=[0.1, 0.1, 0.1])

    # Create a rigid body using the collision shape
    body_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_shape_id)

    # # Set the camera position and orientation
    # p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0, 0, 0])

    # Step through the simulation
    for _ in range(1000):
        p.stepSimulation()
        time.sleep(1. / 240.)
    # Disconnect from the simulation
    p.disconnect()

if __name__ == "__main__":
    num_vertices = 6
    scale = 2.0

    # Generate random polygon
    vertices, faces = generate_random_polygon(num_vertices, scale)

    # Create and visualize the PyBullet polygon
    create_pybullet_polygon(vertices, faces)