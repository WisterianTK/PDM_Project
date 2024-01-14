from RRT_test_PID import run as RRTrun
from RRTu_test_PID import run as RRTurun
import numpy as np

from RRT import RRT

# Seed range to test
seed_start = 20
seed_end = 21
gui = True

num_restarts = 5


def main(seed_start, seed_end):
    print(f"STARTED COMPARISON FROM SEED {seed_start} to {seed_end}")
    for seed in range(seed_start, seed_end):
        print(f"------------------------------------------------------ COMPARING SEED {seed}")
        print(f"------------------------------------------------------ RUNNING RRT")
        try:
            RRTrun(run_algo_cb=runRRT, split_n_wp_cb=split_n_wp, seed=seed, gui=gui)
        except:
            print(f"------------------------------------------------------ RRT FAILED SEED {seed}")
        print(f"------------------------------------------------------ COMPARING SEED {seed}")
        print(f"------------------------------------------------------ RUNNING RRT-U")
        # try:
        #     RRTurun(seed=seed, gui=gui)
        # except:
        #     print(f"------------------------------------------------------ RRT-U FAILED SEED {seed}")


if __name__ == "__main__":
    main(seed_start, seed_end)

def split_n_wp(rrt, NUM_WP):
    #Trajectory = []
    trajectory_has_computed = True
    num_segments = len(rrt.path_to_goal) - 1
    num_points_per_segment = []
    waypoints = []
    
    pathlengths = []
    for index, node in enumerate(rrt.path_to_goal[1:], start=1):
        dist = np.linalg.norm(node - rrt.path_to_goal[index-1])
        pathlengths = dist

    total_length = sum(pathlengths)
    # Calculate number of waypoints for each segment
    for length in pathlengths:
        num_points = round(length / total_length * NUM_WP)
        num_points_per_segment.append(num_points)

    # Adjust for rounding errors
    while sum(num_points_per_segment) > NUM_WP:
        max_index = num_points_per_segment.index(max(num_points_per_segment))
        num_points_per_segment[max_index] -= 1
    while sum(num_points_per_segment) < NUM_WP:
        min_index = num_points_per_segment.index(min(num_points_per_segment))
        num_points_per_segment[min_index] += 1
    

    for index, node in enumerate(rrt.path_to_goal[1:], start=1):
        # Divide one edge into waypoints
        for wp in np.linspace(start=rrt.path_to_goal[index-1], stop=node, num=num_points_per_segment[index-1]):
            waypoints.append(wp)
        # for i in range(Trajectory[-1].shape[0]-1):
        #     p.addUserDebugLine(lineFromXYZ=Trajectory[-1][i], lineToXYZ=Trajectory[-1][i+1], lineColorRGB=[0, 1, 0], lineWidth=2)                   
    
    # TODO Check numwp
    print(f"NUMBER OF WAYPOINTS: {len(waypoints)}")
    return waypoints

def runRRT(obstacles, INIT_POSITION, GOAL_POSITION):
    rrt = RRT(init_node=INIT_POSITION, goal_node=GOAL_POSITION, drone_radius=0.2)
    for i in range(num_restarts):
        goal_found = rrt.runAlgorithm(obstacles=obstacles)
        
        if goal_found:
            #log_data['planner_time'] = time.time() - log_data['start_time']
            break
    return rrt, goal_found





    # elif not has_visualized:
    #         # print("Path found")
    #         # Remove all edges
    #         if not removed:
    #             p.removeAllUserDebugItems()
    #             removed = True
    #         # Add Goal text again
    #         textID = p.addUserDebugText(text="GOAL", textPosition=GOAL_POSITION, textColorRGB=[0, 0, 0], textSize=3)
    #         # Add the path to the goal
    #         # for index, node in enumerate(rrt.path_to_goal[1:], start=1):
    #         #     p.addUserDebugLine(lineFromXYZ=rrt.path_to_goal[index - 1], lineToXYZ=node, lineColorRGB=[0, 1, 0],
    #         #                        lineWidth=2)
    #         has_visualized=True