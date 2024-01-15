# Planning and decision making final project
Tatsuki Fujioka, 5849837; David Schep, 5643384; Dielof van Loon, 5346894

# Install
Install https://github.com/utiasDSL/gym-pybullet-drones using their installation instructions:
```bash
git clone https://github.com/utiasDSL/gym-pybullet-drones.git
cd gym-pybullet-drones/

conda create -n drones python=3.10
conda activate drones

pip3 install --upgrade pip
pip3 install -e . # if needed, `sudo apt install build-essential` to install `gcc` and build `pybullet`

```

# Run
Always activate the conda environment before running code: `conda activate drones`.

To run the main script that generates the data used in the project:
```bash
python3 compare_algos.py
```
This runs 200 simulations. 100 for RRT and 100 for RRT-u. This takes approximately 20 minutes, depending on processing power.

The `compare_algo.py` script has a few command line parameters:
```
  --seed_start SEED_START
                        The start seed for the tests. (default=1240918)
  --num_seeds NUM_SEEDS
                        The number of seeds to test. (default=100)
  --gui GUI             Whether to use GUI or not.
  --num_restarts NUM_RESTARTS
                        The number of restarts each algorithm is allowed. (default=0)
  --num_obstacles NUM_OBSTACLES
                        The number of obstacles. (default=30)
  --sample_box SAMPLE_BOX
                        The sample box for the algorithms. (default=((-2, -2, 0), (6, 6, 4)))
```
For example, to run a single seed, with gui:
```
python3 compare_algos.py --gui --num_seeds 1 --seed_start 1234
```
This runs both RRT and RRT-u on the same seed, once.

## RRT and RRT-u visualization scripts
Both rrt and rrt-u have a visualization script. They can be run using the following commands:
```
python3 vis_RRT.py
python3 vis_RRTu.py
```

These scripts only run the path planning algorithm and do not fly the drone along the path. They are used to collect images and videos for the report and presentation.

## File structure
- `Obstacles.py`: Contains the Obstacles class, generates the random obstacles.
- `util.py`: Contains utility functions.
- `create_graphs.ipynb`: A jupyter notebook to generate a graph for the report.
- `all_test_PID.py`: Contains the simulator. This simulator function takes callback functions that calculate the path using RRT or RRT-u. 
- `RRT.py`: Contains the RRT class, uses the RRT algorithm to generate a path.
- `RRTu.py`: Contains the RRT-u class, uses the RRT-u algorithm to generate a path.
- `vis_RRT.py`: Used te create visualizations of RRT
- `vis_RRTu.py`: Used te create visualizations of RRT-u