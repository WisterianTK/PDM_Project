from compare_algos import split_n_wp_rrtu, split_n_wp_rrt, runRRTu, runRRT
from all_test_PID import run

gui = True
speed_multi = 30
num_obstacles = 30

_, _, _, _, _ = run(runRRT, 
                    split_n_wp_rrt, 
                    seed=1, 
                    gui=gui, 
                    SPEED_MULTIPLIER=30, # Lower speedmulti is faster
                    # Max speed, used in report: 20. 93% flight success rate
                    num_obstacles=num_obstacles) 