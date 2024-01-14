import pybullet as p
import time
import json
import pybullet_data
import scipy as sp
import numpy as np
from Obstacles import RandomObstacles

from util import write_json, drawPoint, elapsedTime

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_NUM_DRONES = 1
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = True
DEFAULT_RECORD_VISION = False
DEFAULT_PLOT = True
DEFAULT_USER_DEBUG_GUI = False
DEFAULT_OBSTACLES = False
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_DURATION_SEC = 30
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False
DEFAULT_SEED = 6



def run(
        run_algo_cb,
        split_n_wp_cb,
        drone=DEFAULT_DRONES,
        num_drones=DEFAULT_NUM_DRONES,
        physics=DEFAULT_PHYSICS,
        gui=DEFAULT_GUI,
        record_video=DEFAULT_RECORD_VISION,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        obstacles=DEFAULT_OBSTACLES,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        duration_sec=DEFAULT_DURATION_SEC,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        colab=DEFAULT_COLAB,
        seed=DEFAULT_SEED,
        SPEED_MULTIPLIER=40,
        GOAL_POSITION = np.array([4, 4, 1.5])
        ):
    #### Initialize the simulation #############################
    INIT_POSITION = np.array([[0, 0, 1]])
    INIT_ORIENTATION = np.array([[0, 0, 0]])
    np.random.seed(seed)

    #### Create the environment ################################
    env = CtrlAviary(drone_model=drone,
                     num_drones=num_drones,
                     initial_xyzs=INIT_POSITION,
                     initial_rpys=INIT_ORIENTATION,
                     physics=physics,
                     neighbourhood_radius=10,
                     pyb_freq=simulation_freq_hz,
                     ctrl_freq=control_freq_hz,
                     gui=gui,
                     record=record_video,
                     obstacles=obstacles,
                     user_debug_gui=user_debug_gui
                     )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    # Set Goal position

    drawPoint(INIT_POSITION[0], [0,1,0], 0.2)
    drawPoint(GOAL_POSITION, [1,0,0], 0.2)


    # Set number of convex obstacles
    num_obstacles = 30


    # Generate convex obstacles
    obstacles = RandomObstacles(num_obstacles=num_obstacles, goal_position=GOAL_POSITION, 
                                initial_position=INIT_POSITION,
                                bounding_box=[[-2, -2, 0],[6, 6, 4]])

    # Initialize RRT
    algo, goal_found = run_algo_cb(obstacles, INIT_POSITION[0], GOAL_POSITION)
    if not goal_found:
        raise Exception("GOAL NOT FOUND")
    
    # Calculate NUM_WP based on length of path
    NUM_WP = round(algo.path_length * SPEED_MULTIPLIER)


    waypoints = split_n_wp_cb(algo, NUM_WP)
    assert len(waypoints) == NUM_WP

    for i in range(0,len(waypoints),20):
        drawPoint(waypoints[i],[0.0,0.0,1.0], 0.04)

    #### Initialize trajectory related parameters
    # Number of waypoints between two nodes, reducing NUM_WP makes drone move faster
    #NUM_WP = int(control_freq_hz/2)
    # NUM_WP = control_freq_hz
    # Target positions for one control period
    TARGET_POS = np.zeros((NUM_WP,3))

    # # Stay at the initial position
    for i in range(NUM_WP):
        TARGET_POS[i, :] = waypoints[i]
    wp_counters = np.array([0])

    # #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=control_freq_hz,
                    num_drones=num_drones,
                    output_folder=output_folder,
                    colab=colab
                    )

    #### Initialize the controllers ############################
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:
        ctrl = [DSLPIDControl(drone_model=drone)]


    #### Run the simulation ####################################
    action = np.zeros((1, 4))
    log_data = dict()
    

    drone_realtime = 0.0
    drone_simtime = 0.0
    distance_traveled = 0.0
    total_error = 0.0

    goal_reached = False
    # Add text GOAl in simulation
    textID = p.addUserDebugText(text="GOAL", textPosition=GOAL_POSITION, textColorRGB=[0, 0, 0], textSize=1)

    # Set camera postion
    camera_target_position = GOAL_POSITION  # Target position the camera is looking at
    camera_distance = 3.0  # Distance from the target position
    camera_yaw = 80  # Yaw angle in degrees
    camera_pitch = -30  # Pitch angle in degrees

    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=camera_target_position,
    )

    previous_location = INIT_POSITION[0]
    print("Starting flight sim")
    ####### SIM ################
    START = time.time()
    drone_cycles = 0
    for i in range(0, int(duration_sec*env.CTRL_FREQ)): #int(duration_sec*env.CTRL_FREQ)
        drone_cycles += 1
        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### Compute control for the current way point #############
        action[0, :], position_error, _ = ctrl[0].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                state=obs[0],
                                                                target_pos=TARGET_POS[wp_counters[0],:],
                                                            )
        
        # Collect total error for comparison
        total_error += position_error

        distance_traveled += np.linalg.norm(obs[0][0:3] - previous_location)

        #### Go to the next way point and loop #####################
        if wp_counters[0] < NUM_WP-1:
            wp_counters[0] = wp_counters[0] + 1


        if np.linalg.norm(GOAL_POSITION-obs[0,:3]) <= 0.2:
            print("GOAL REACHED")
            goal_reached = True
            drone_realtime = elapsedTime(START)
            drone_simtime = drone_cycles / env.CTRL_FREQ
            break
        
        #### Log the simulation ####################################
        logger.log(drone=0,
                    timestamp=i/env.CTRL_FREQ,
                    state=obs[0],
                    )
        
        #### Printout ##############################################
        #env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)
    
        previous_location = obs[0][0:3]
    #### Close the environment #################################
    
    # TODO
    env.close()
    return goal_reached, drone_realtime, drone_simtime, total_error.tolist(), distance_traveled
    # #### Save the simulation results ###########################
    #logger.save()
    #logger.save_as_csv("pid") # Optional CSV save
    #
    # #### Plot the simulation results ###########################
    # if plot:
    #     logger.plot()

if __name__ == "__main__":
    run()
