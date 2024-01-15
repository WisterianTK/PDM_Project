from compare_algos import split_n_wp_rrtu, split_n_wp_rrt, runRRTu, runRRT
from all_test_PID import run
from util import drawPolynomial
import numpy as np
import time
from RRT import RRT
import pybullet as p

gui = True
speed_multi = 30
num_obstacles = 30


# Callback function that runs the rrt algorithm
def runRRT(obstacles, INIT_POSITION, GOAL_POSITION):
    print(f"Running rrt: {INIT_POSITION, GOAL_POSITION}")
    # Initialize RRT
    rrt = RRT(init_node=INIT_POSITION, goal_node=GOAL_POSITION, drone_radius=0.2, max_iter=300, time_limit=np.Inf, config_box=((-2, -2, 0), (6, 6, 4)))
    for i in range(1):
        # Run RRT
        goal_found = rrt.runAlgorithm(obstacles=obstacles)
        
        if goal_found:
            print("Goal found")
            break
    # Return RRT object and boolean goal_found
        
    # Remove obstacles again after running the algorithm
    for obs in obstacles.obstacleIDs:
        p.removeBody(obs)
    return rrt, goal_found


# Callback function that splits the found rrt-u path into NUM_WP waypoints
def draw_lines_and_sleep(rrt, NUM_WP):
    # Trace the found path
    # Store in a list of Nodes

    path = rrt.path_to_goal

    for i in range(1,len(path)):
        p.addUserDebugLine(path[i-1], path[i], [0,1,0], lineWidth=2)

    for path in rrt.paths:
        p.addUserDebugLine(path[0], path[1], [0,0,1], lineWidth=1)

    time.sleep(10000)
    raise Exception("Break")
    return waypoints



_, _, _, _, _ = run(runRRT, 
                    draw_lines_and_sleep, 
                    seed=1, 
                    gui=gui, 
                    SPEED_MULTIPLIER=30, # Lower speedmulti is faster
                    # Max speed, used in report: 20. 93% flight success rate
                    num_obstacles=num_obstacles) 