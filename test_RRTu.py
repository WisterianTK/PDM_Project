from compare_algos import split_n_wp_rrtu, split_n_wp_rrt, runRRTu, runRRT
from all_test_PID import run
from util import drawPolynomial
import numpy as np
import time
from RRTu import RRTu
gui = True
speed_multi = 30
num_obstacles = 30


# Callback function that runs the rrt-u algorithm
def runRRTu(obstacles, INIT_POSITION, GOAL_POSITION):
    print(f"Running rrt: {INIT_POSITION, GOAL_POSITION}")
    # Initialize RRT-u
    rrtu = RRTu(init_position=INIT_POSITION, goal_position=GOAL_POSITION, step_size_delta_time=0.4, max_iter=80, time_limit=np.inf, drone_radius=0.2, config_box=((-2, -2, 0), (6, 6, 4)))
    for i in range(1):
        # Run RRT-u
        goal_found = rrtu.runAlgorithm(obstacles=obstacles)

        if goal_found:
            print("Goal found")
            break
    # Return RRTu object and boolean goal_found
    return rrtu, goal_found


# Callback function that splits the found rrt-u path into NUM_WP waypoints
def draw_lines_and_sleep(rrtu, NUM_WP):
    # Trace the found path
    # Store in a list of Nodes
    path_nodes = list(rrtu.tracePath())

    for i in range(1, len(path_nodes)):
        drawPolynomial(path_nodes[i-1].position, path_nodes[i-1].velocities, path_nodes[i].accelerations, path_nodes[i].dt, color=[0,1,0], num_segments=10, line_size=2)

    for node in rrtu.nodes:
        if node.parent:
            drawPolynomial(node.parent.position, node.parent.velocities, node.accelerations, node.dt, color=[0,0,1], num_segments=10, line_size=1)

    time.sleep(10000)
    raise Exception("Break")
    return waypoints



_, _, _, _, _ = run(runRRTu, 
                    draw_lines_and_sleep, 
                    seed=1, 
                    gui=gui, 
                    SPEED_MULTIPLIER=30, # Lower speedmulti is faster
                    # Max speed, used in report: 20. 93% flight success rate
                    num_obstacles=num_obstacles) 