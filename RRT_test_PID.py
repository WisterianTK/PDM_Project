import pybullet as p
import time
import pybullet_data
import scipy as sp
import numpy as np
from Obstacles import RandomObstacles
from RRT import RRT

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
DEFAULT_DURATION_SEC = 40
DEFAULT_OUTPUT_FOLDER = 'results'
DEFAULT_COLAB = False






def run(
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
        colab=DEFAULT_COLAB
        ):

    #### Initialize the simulation #############################
    INIT_POSITION = np.array([[0, 0, 1]])
    INIT_ORIENTATION = np.array([[0, 0, 0]])

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
    GOAL_POSITION = np.array([3, 3, 2])

    # Set number of convex obstacles
    num_obstacles = 100

    # Generate convex obstacles
    obstacles = RandomObstacles(num_obstacles=num_obstacles, goal_position=GOAL_POSITION, initial_position=INIT_POSITION)

    # Initialize RRT
    rrt = RRT(init_node=np.array([0, 0, 1]), goal_node=GOAL_POSITION)

    #### Initialize trajectory related parameters
    # Number of waypoints between two nodes
    NUM_WP = 100
    # Target positions for one control period
    TARGET_POS = np.zeros((NUM_WP,3))

    # Stay at the initial position
    for i in range(NUM_WP):
        TARGET_POS[i, :] = INIT_POSITION[0, 0], INIT_POSITION[0, 1], INIT_POSITION[0, 2]
    wp_counters = np.array([0])

    # #### Initialize the logger #################################
    # logger = Logger(logging_freq_hz=control_freq_hz,
    #                 num_drones=num_drones,
    #                 output_folder=output_folder,
    #                 colab=colab
    #                 )

    #### Initialize the controllers ############################
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:
        ctrl = [DSLPIDControl(drone_model=drone)]


    #### Run the simulation ####################################
    action = np.zeros((1, 4))
    START = time.time()
    goal_found = False
    removed = False
    trajectory_has_computed = False
    has_visualized = False
    goal_reached = False
    # Add text GOAl in simulation
    textID = p.addUserDebugText(text="GOAL", textPosition=GOAL_POSITION, textColorRGB=[0, 0, 0], textSize=1)

    for i in range(0, int(duration_sec*env.CTRL_FREQ)):
        #### Step the simulation ###################################
        obs, reward, terminated, truncated, info = env.step(action)

        #### Run RRT
        if not goal_found:
            goal_found = rrt.runAlgorithm(obstacles=obstacles)
            for path in rrt.paths:
                p.addUserDebugLine(lineFromXYZ=path[0], lineToXYZ=path[1], lineColorRGB=[1, 0, 0])
        # Remove all edges and visualize the path to the goal
        elif not has_visualized:
            # print("Path found")
            # Remove all edges
            if not removed:
                p.removeAllUserDebugItems()
                removed = True
            # Add Goal text again
            textID = p.addUserDebugText(text="GOAL", textPosition=GOAL_POSITION, textColorRGB=[0, 0, 0], textSize=1)
            # Add the path to the goal
            for index, node in enumerate(rrt.path_to_goal[1:], start=1):
                p.addUserDebugLine(lineFromXYZ=rrt.path_to_goal[index - 1], lineToXYZ=node, lineColorRGB=[0, 1, 0],
                                   lineWidth=2)
            has_visualized=True
        # Stay at initial position if goal path has not been found
        if not goal_found:
            action[0, :], _, _ = ctrl[0].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                 state=obs[0],
                                                                 target_pos=np.hstack([TARGET_POS[wp_counters[0], 0:2],
                                                                                      INIT_POSITION[0, 2]]),
                                                                 target_rpy=INIT_ORIENTATION[0, :]
                                                                 )

        # Follow the path to the goal
        if goal_found and not goal_reached:
            if not trajectory_has_computed:
                Trajectory = []
                trajectory_has_computed = True
                for index, node in enumerate(rrt.path_to_goal[1:], start=1):
                    # Divide one edge into waypoints
                    Trajectory.append(np.linspace(start=rrt.path_to_goal[index-1], stop=node, num=NUM_WP))
                trajectory_counter = 0

            if trajectory_counter < len(Trajectory):
                #### Compute control for the current way point #############
                action[0, :], _, _ = ctrl[0].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                     state=obs[0],
                                                                     target_pos=Trajectory[trajectory_counter][wp_counters[0],:],
                                                                    )
                if not wp_counters[0] < (NUM_WP-1):
                    trajectory_counter += 1
                if trajectory_counter == len(Trajectory) and wp_counters==NUM_WP-1:
                    goal_reached = True

                #### Go to the next way point and loop #####################
                wp_counters[0] = wp_counters[0] + 1 if wp_counters[0] < (NUM_WP-1) else 0
        if goal_reached:
            print("goal reached")
            action[0, :], _, _ = ctrl[0].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                 state=obs[0],
                                                                 target_pos=GOAL_POSITION,
                                                                 )


        #### Log the simulation ####################################
        # for j in range(num_drones):
        #
            # logger.log(drone=j,
            #            timestamp=i/env.CTRL_FREQ,
            #            state=obs[j],
            #            control=np.hstack([TARGET_POS[wp_counters[j], 0:2], INIT_POSITION[j, 2], INIT_ORIENTATION[j, :], np.zeros(6)])
            #            )

        #### Printout ##############################################
        # env.render()

        #### Sync the simulation ###################################
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)

    #### Close the environment #################################
    env.close()

    # #### Save the simulation results ###########################
    # logger.save()
    # logger.save_as_csv("pid") # Optional CSV save
    #
    # #### Plot the simulation results ###########################
    # if plot:
    #     logger.plot()

if __name__ == "__main__":

    run()
