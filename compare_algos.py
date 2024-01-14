from all_test_PID import run
import numpy as np

from RRT import RRT
from RRTu import RRTu
from util import write_json
# Seed range to test
seed_start = 1240918
num_seeds = 100
gui = False

num_runs = 1

sample_box=((-2, -2, 0), (6, 6, 4))


def split_n_wp_rrt(rrt, NUM_WP):
    #Trajectory = []

    num_points_per_segment = []
    waypoints = []
    
    pathlengths = []
    for index, node in enumerate(rrt.path_to_goal[1:], start=1):
        dist = np.linalg.norm(node - rrt.path_to_goal[index-1])
        pathlengths.append(dist)

    total_length = np.sum(pathlengths)
    print(f"Total len: {total_length}")
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

# Currently using num_segments, a step_size would be better (more consistent distances between samples)
# However for step_size we would need the arclenght which is expensive to compute
def sample_polynomial_on_velocity(start_position, start_velocity, acceleration, delta_t_total, num_samples=30):
    # generator for sampling points along a given polynomial
    assert len(start_position) == 3, "Input should be a numpy array of size 3"
    assert len(start_velocity) == 3, "Input should be a numpy array of size 3"
    assert len(acceleration) == 3, "Input should be a numpy array of size 3"

    # Calculate the time step
    delta_time_segment = delta_t_total / num_samples

    # Calculate the position at each time step
    for i in range(1, num_samples+1):
        t = i * delta_time_segment
        yield start_position + start_velocity * t + 0.5 * acceleration * t**2

def split_n_wp_rrtu(rrtu, NUM_WP):
    path_nodes = list(rrtu.tracePath())

    num_points_per_segment = []
    waypoints = []
    
    pathtimes = []
    for index in range(1,len(path_nodes)):
        pathtimes.append(path_nodes[index].dt)

    total_time = np.sum(pathtimes)
    print(f"RRTU Total time: {total_time}")
    # Calculate number of waypoints for each segment
    for path_time in pathtimes:
        num_points = round(path_time / total_time * NUM_WP)
        num_points_per_segment.append(num_points)

    # Adjust for rounding errors
    while sum(num_points_per_segment) > NUM_WP:
        max_index = num_points_per_segment.index(max(num_points_per_segment))
        num_points_per_segment[max_index] -= 1
    while sum(num_points_per_segment) < NUM_WP:
        min_index = num_points_per_segment.index(min(num_points_per_segment))
        num_points_per_segment[min_index] += 1
    print("TOTAL NUM POINTS:", np.sum(num_points_per_segment), NUM_WP)
    for i in range(len(path_nodes)-1):
        # Divide one edge into waypoints
        for point in sample_polynomial_on_velocity(path_nodes[i].position, path_nodes[i].velocities, path_nodes[i+1].accelerations, path_nodes[i+1].dt, num_samples=num_points_per_segment[i]):
            waypoints.append(point)

    # TODO Check numwp
    print(f"RRTU| NUMBER OF WAYPOINTS: {len(waypoints)}")
    return waypoints

def runRRT(obstacles, INIT_POSITION, GOAL_POSITION):
    print(f"Running rrt: {INIT_POSITION, GOAL_POSITION}")
    rrt = RRT(init_node=INIT_POSITION, goal_node=GOAL_POSITION, drone_radius=0.2, max_iter=300, time_limit=4.0, config_box=sample_box)
    for i in range(num_runs):
        goal_found = rrt.runAlgorithm(obstacles=obstacles)
        
        if goal_found:
            print("Goal found")
            #log_data['planner_time'] = time.time() - log_data['start_time']
            break
    return rrt, goal_found

def runRRTu(obstacles, INIT_POSITION, GOAL_POSITION):
    print(f"Running rrt: {INIT_POSITION, GOAL_POSITION}")
        # Initialize RRT
    rrt = RRTu(init_position=INIT_POSITION, goal_position=GOAL_POSITION, step_size_delta_time=0.4, max_iter=300, time_limit=4.0, drone_radius=0.2, config_box=sample_box)
    for i in range(num_runs):
        goal_found = rrt.runAlgorithm(obstacles=obstacles)
        
        if goal_found:
            print("Goal found")
            #log_data['planner_time'] = time.time() - log_data['start_time']
            break
    return rrt, goal_found


def main():
    print(f"STARTED COMPARISON FROM SEED {seed_start} to {seed_start+num_seeds}")
    datarrt = []
    datarrtu = []
    
    for seed in range(seed_start, seed_start+num_seeds):
        print(f"------------------------------------------------------ COMPARING SEED {seed}")
        print(f"------------------------------------------------------ RUNNING RRT")
        try:
            goal_reached, drone_realtime, drone_simtime, total_error, distance_traveled = run(runRRT, split_n_wp_rrt, seed=seed, gui=gui, SPEED_MULTIPLIER=20) # Lower speedmulti is faster
            datarrt.append({"goal_found": True,
                            "goal_reached": goal_reached,
                            "drone_realtime": drone_realtime,
                            "drone_simtime": drone_simtime,
                            "total_error": total_error,
                            "distance_traveled": distance_traveled})
        except:
            print("RRT GOAL NOT REACHED")
            datarrt.append({"goal_found": False,
                            "goal_reached": False,
                            "drone_realtime": np.inf,
                            "drone_simtime": np.inf,
                            "total_error": np.inf,
                            "distance_traveled": np.inf})
            print(f"------------------------------------------------------ RRT FAILED SEED {seed}")
        print(f"------------------------------------------------------ COMPARING SEED {seed}")
        print(f"------------------------------------------------------ RUNNING RRT-U")
        
        try:
            goal_reached, drone_realtime, drone_simtime, total_error, distance_traveled = run(runRRTu, split_n_wp_rrtu, seed=seed, gui=gui, SPEED_MULTIPLIER=18) # Lower speedmulti is faster
            datarrtu.append({"goal_found": True,
                            "goal_reached": goal_reached,
                            "drone_realtime": drone_realtime,
                            "drone_simtime": drone_simtime,
                            "total_error": total_error,
                            "distance_traveled": distance_traveled})
        except Exception as e:
            print("RRTU EXCEPTION: ", str(e))
            print("RRTU GOAL NOT REACHED")
            datarrtu.append({"goal_found": False,
                            "goal_reached": False,
                            "drone_realtime": np.inf,
                            "drone_simtime": np.inf,
                            "total_error": np.inf,
                            "distance_traveled": np.inf})
            print(f"------------------------------------------------------ RRTU FAILED SEED {seed}")
        # try:
        #
        ## except:
        #     print(f"------------------------------------------------------ RRT-U FAILED SEED {seed}")

    print(datarrt)
    print(datarrtu)

    def calcAverages(data):
        percentageGoalFound = np.mean([x['goal_found'] for x in data])
        percentageGoalReachedWhenGoalFound = np.mean([x['goal_reached'] for x in data if x['goal_found']])
        averageDroneRealtime = np.mean([x['drone_realtime'] for x in data if x['goal_found'] and x['goal_reached']])
        averageDroneSimtime = np.mean([x['drone_simtime'] for x in data if x['goal_found'] and x['goal_reached']])
        averageErrorX = np.mean([x['total_error'][0] for x in data if x['goal_found'] and x['goal_reached']])
        averageErrorY = np.mean([x['total_error'][1] for x in data if x['goal_found'] and x['goal_reached']])
        averageErrorZ = np.mean([x['total_error'][2] for x in data if x['goal_found'] and x['goal_reached']])
        averageDistance = np.mean([x['distance_traveled'] for x in data if x['goal_found'] and x['goal_reached']])
        return {"percentageGoalFound": percentageGoalFound,
                "percentageGoalReachedWhenGoalFound": percentageGoalReachedWhenGoalFound,
                "averageDroneRealtime": averageDroneRealtime,
                "averageDroneSimtime": averageDroneSimtime,
                "averageErrorX": averageErrorX,
                "averageErrorY": averageErrorY,
                "averageErrorZ": averageErrorZ,
                "averageDistance": averageDistance,
                }
    averages = {}
    print("---------------RRT------------------")
    averages['RRT'] = calcAverages(datarrt)
    print(averages['RRT'])
    print("---------------RRT-U------------------")
    averages['RRTU'] = calcAverages(datarrtu)
    print(averages['RRTU'])

    write_json(datarrt, filename = 'rrt_data_NEW.json')
    write_json(datarrtu, filename = 'rrtu_data_NEW.json')
    write_json(averages, filename = 'averages_NEW.json')

if __name__ == "__main__":
    main()


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