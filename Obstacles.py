# Obstacles class which generates obstacles randomly
import scipy as sp
import pybullet as p
import numpy as np


class RandomObstacles:
    def __init__(self, num_obstacles, goal_position, range_num_vertices=[7, 8], bounding_box=[[-10, -10, 0],[10, 10, 3]], meshScale=0.1):
        # num_obstacles: Number of obstacles
        # goal_position: Position of goal in work space
        # range_num_vertices: Range of number of vertices from which the number of vertices is sampled for each obstacle
        # bounding_box: Bounding box in which you want to generate random obstacles
        # convexHulls: List that stores all convex hulls
        # basePositions: List that stores base positions of convex hulls
        self.obstacleIDs = list()
        self.collisionShapeIDs = list()
        self.convexHulls= list()
        self.basePositions = list()
        self.num_obstacles = num_obstacles
        self.meshScale = meshScale



        # Initialize random obstacles
        for i in range(num_obstacles):
            # Random number of vertices
            num_vertices = np.random.randint(range_num_vertices[0], range_num_vertices[1])

            # Create points of convex hull
            points = create_convex_hull(num_vertices)

            # Create convex hull (the same points are concatenated to improve a rending issue)
            hull = sp.spatial.ConvexHull(np.concatenate((points, points, points), axis=0))
            vertices = hull.points

            # Store the created convex hull
            self.convexHulls.append(hull)


            # indices = hull.simplices.flatten()
            # # Create visual shape and add to the list
            # visualShapeID = p.createVisualShape(shapeType=p.GEOM_MESH,
            #                                      rgbaColor=[1, 0, 0, 1],
            #                                      specularColor=[0.4, 0.4, 0],
            #                                      meshScale=self.meshScale*np.array([1., 1., 1.]),
            #                                      vertices=vertices,
            #                                      indices=indices)
            # self.visualShapeIDs.append(visualShapeID)

            # Create collision shape and add to the list
            collisionShapeID = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                                       vertices=vertices,
                                                       meshScale=self.meshScale*np.array([1., 1., 1.]))
            self.collisionShapeIDs.append(collisionShapeID)

            # Create obstacle and place it within the bounding box of the world and avoid placing too close to the goal
            # (1.5 of margin around goal position)

            # Sample point within the bounding box
            basePosition = np.array([np.random.randint(bounding_box[0][0], bounding_box[1][0]),
                            np.random.randint(bounding_box[0][1], bounding_box[1][1]),
                            np.random.randint(bounding_box[0][2], bounding_box[1][2])])

            # Condition: the distance between base position of obstacle and goal should be larger than 1
            condition = np.linalg.norm(basePosition-goal_position) <= 1

            while condition:
                basePosition = np.array([np.random.randint(bounding_box[0][0], bounding_box[1][0]),
                                         np.random.randint(bounding_box[0][1], bounding_box[1][1]),
                                         np.random.randint(bounding_box[0][2], bounding_box[1][2])])
                condition = np.linalg.norm(basePosition - goal_position) <= 1

            # Save the base position
            self.basePositions.append(basePosition)
            obstacleID = p.createMultiBody(baseCollisionShapeIndex=collisionShapeID,
                                           basePosition=basePosition)
            self.obstacleIDs.append(obstacleID)


# Function to create random convex shape
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
