import argparse
from all_test_PID import run
import numpy as np

from RRT import RRT
from RRTu import RRTu
from util import write_json

# Create the parser
parser = argparse.ArgumentParser(description="Compare algorithms script")

# Add the arguments
parser.add_argument('--seed_start', type=int, default=1234567, help='The start seed for the tests.')
parser.add_argument('--num_seeds', type=int, default=5, help='The number of seeds to test.')
parser.add_argument('--gui', type=bool, default=True, help='Whether to use GUI or not.')
parser.add_argument('--num_restarts', type=int, default=0, help='The number of restarts each algorithm is allowed.')
parser.add_argument('--num_obstacles', type=int, default=30, help='The number of obstacles.')
parser.add_argument('--sample_box', type=tuple, default=((-2, -2, 0), (6, 6, 4)), help='The sample box for the algorithms.')

# Parse the arguments
args = parser.parse_args()

# Set the global variables
seed_start = args.seed_start
num_seeds = args.num_seeds
gui = args.gui
num_runs = args.num_restarts + 1
num_obstacles = args.num_obstacles
sample_box = args.sample_box

# Callback function that runs the rrt algorithm
def runRRT(obstacles, INIT_POSITION, GOAL_POSITION):
    print(f"Running rrt: {INIT_POSITION, GOAL_POSITION}")
    # Initialize RRT
    rrt = RRT(init_node=INIT_POSITION, goal_node=GOAL_POSITION, drone_radius=0.2, max_iter=300, time_limit=4.0, config_box=sample_box)
    for i in range(num_runs):
        # Run RRT
        goal_found = rrt.runAlgorithm(obstacles=obstacles)
        
        if goal_found:
            print("Goal found")
            break
    # Return RRT object and boolean goal_found
    return rrt, goal_found

# Callback function that runs the rrt-u algorithm
def runRRTu(obstacles, INIT_POSITION, GOAL_POSITION):
    print(f"Running rrt: {INIT_POSITION, GOAL_POSITION}")
    # Initialize RRT-u
    rrtu = RRTu(init_position=INIT_POSITION, goal_position=GOAL_POSITION, step_size_delta_time=0.4, max_iter=300, time_limit=4.0, drone_radius=0.2, config_box=sample_box)
    for i in range(num_runs):
        # Run RRT-u
        goal_found = rrtu.runAlgorithm(obstacles=obstacles)

        if goal_found:
            print("Goal found")
            break
    # Return RRTu object and boolean goal_found
    return rrtu, goal_found

# Callback function that splits the found rrt path into NUM_WP waypoints
def split_n_wp_rrt(rrt, NUM_WP):
    num_points_per_segment = []
    waypoints = []  
    pathlengths = []
    # Calculate distances between each 2 following nodes
    # Store in pathlengths
    for index, node in enumerate(rrt.path_to_goal[1:], start=1):
        dist = np.linalg.norm(node - rrt.path_to_goal[index-1])
        pathlengths.append(dist)
    # Calculate total length of the path
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

    print(f"Number of waypoints: {len(waypoints)}")
    return waypoints

# Function to sample a polynomial, the time between 2 samples is always equal
# num_samples: Number of samples in delta_t_total
def sample_polynomial_on_velocity(start_position, start_velocity, acceleration, delta_t_total, num_samples=30):
    assert len(start_position) == 3, "Input should be a numpy array of size 3"
    assert len(start_velocity) == 3, "Input should be a numpy array of size 3"
    assert len(acceleration) == 3, "Input should be a numpy array of size 3"

    # Calculate the time step
    delta_time_segment = delta_t_total / num_samples

    # Calculate the position at each time step
    for i in range(1, num_samples+1):
        t = i * delta_time_segment
        yield start_position + start_velocity * t + 0.5 * acceleration * t**2

# Callback function that splits the found rrt-u path into NUM_WP waypoints
def split_n_wp_rrtu(rrtu, NUM_WP):
    # Trace the found path
    # Store in a list of Nodes
    path_nodes = list(rrtu.tracePath())

    num_points_per_segment = []
    waypoints = [] 
    pathtimes = []
    # retrieve the dt for each segment
    for index in range(1,len(path_nodes)):
        pathtimes.append(path_nodes[index].dt)

    # Total time of the entire path
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

    print(f"RRTU| NUMBER OF WAYPOINTS: {len(waypoints)}")
    return waypoints

# Run num_seeds amount of experiments
# Each time use same seed for both algorithms
# Then increase seed by 1
def main():
    print(f"STARTED COMPARISON FROM SEED {seed_start} to {seed_start+num_seeds}")
    # Lists to store data in
    datarrt = []
    datarrtu = []
    
    for seed in range(seed_start, seed_start+num_seeds):
        print(f"COMPARING SEED {seed}")
        print(f"RUNNING RRT")
        try:
            # Run the rrt algorithm though the simulator
            goal_reached, drone_realtime, drone_simtime, total_error, distance_traveled = run(runRRT, 
                                                                                              split_n_wp_rrt, 
                                                                                              seed=seed, 
                                                                                              gui=gui, 
                                                                                              SPEED_MULTIPLIER=30, # Lower speedmulti is faster
                                                                                              # Max speed, used in report: 20. 93% flight success rate
                                                                                              num_obstacles=num_obstacles) 
            # Store data
            datarrt.append({"goal_found": True,
                            "goal_reached": goal_reached,
                            "drone_realtime": drone_realtime,
                            "drone_simtime": drone_simtime,
                            "total_error": total_error,
                            "distance_traveled": distance_traveled})
        except Exception as e:
            print("RRT GOAL NOT REACHED")
            # Goal has not been reached
            datarrt.append({"goal_found": False,
                            "goal_reached": False,
                            "drone_realtime": np.inf,
                            "drone_simtime": np.inf,
                            "total_error": np.inf,
                            "distance_traveled": np.inf})
            print(f"RRT FAILED SEED {seed}")
        print(f"COMPARING SEED {seed}")
        print(f"RUNNING RRT-U")
        
        try:
            # Run the rrt-u algorithm though the simulator
            goal_reached, drone_realtime, drone_simtime, total_error, distance_traveled = run(runRRTu, 
                                                                                              split_n_wp_rrtu, 
                                                                                              seed=seed, 
                                                                                              gui=gui, 
                                                                                              SPEED_MULTIPLIER=30, # Lower speedmulti is faster
                                                                                              # Max speed, used in report: 18. 93% flight success rate
                                                                                              num_obstacles=num_obstacles) 
            # Store data
            datarrtu.append({"goal_found": True,
                            "goal_reached": goal_reached,
                            "drone_realtime": drone_realtime,
                            "drone_simtime": drone_simtime,
                            "total_error": total_error,
                            "distance_traveled": distance_traveled})
        except Exception as e:
            #print("RRTU EXCEPTION: ", str(e))
            print("RRTU GOAL NOT REACHED")
            # Goal has not been reached
            datarrtu.append({"goal_found": False,
                            "goal_reached": False,
                            "drone_realtime": np.inf,
                            "drone_simtime": np.inf,
                            "total_error": np.inf,
                            "distance_traveled": np.inf})
            print(f"------------------------------------------------------ RRTU FAILED SEED {seed}")

    print(datarrt)
    print(datarrtu)

    # Calculate averages for each of the metrics
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

    # Calculate and print averages
    print("---------------RRT------------------")
    averages['RRT'] = calcAverages(datarrt)
    print(averages['RRT'])
    print("---------------RRT-U------------------")
    averages['RRTU'] = calcAverages(datarrtu)
    print(averages['RRTU'])

    # Write all data to json files
    write_json(datarrt, filename = 'data/rrt_data_NEW.json')
    write_json(datarrtu, filename = 'data/rrtu_data_NEW.json')
    write_json(averages, filename = 'data/averages_NEW.json')

if __name__ == "__main__":
    main()
