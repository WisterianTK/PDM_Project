from RRT_test_PID import run as RRT
from RRTu_test_PID import run as RRTu
import numpy as np

# Seed range to test
seed_start = 0
seed_end = 50


def main(seed_start, seed_end):
    print(f"STARTED COMPARISON FROM SEED {seed_start} to {seed_end}")
    for seed in range(seed_start, seed_end):
        print(f"------------------------------------------------------ COMPARING SEED {seed}")
        print(f"------------------------------------------------------ RUNNING RRT")
        try:
            RRT(seed=seed)
        except:
            print(f"------------------------------------------------------ RRT FAILED SEED {seed}")
        print(f"------------------------------------------------------ COMPARING SEED {seed}")
        print(f"------------------------------------------------------ RUNNING RRT-U")
        try:
            RRTu(seed=seed)
        except:
            print(f"------------------------------------------------------ RRT-U FAILED SEED {seed}")


if __name__ == "__main__":
    main(seed_start, seed_end)