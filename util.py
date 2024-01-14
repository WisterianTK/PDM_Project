import pybullet as p
import numpy as np
import json
import os
from time import time

def elapsedTime(startTime):
    return time() - startTime

def drawPoint(point, color, size=0.1):
    assert len(point) == 3, "Input should be a numpy array of size 3"
    x, y, z = point[0], point[1], point[2]
    #size = 0.1  # Size of the cross
    #color = [0, 0, 0]  # RGB color
    p.addUserDebugLine([x-size, y, z], [x+size, y, z], color)
    p.addUserDebugLine([x, y-size, z], [x, y+size, z], color)
    p.addUserDebugLine([x, y, z-size], [x, y, z+size], color)


# def drawPolynomial(start_position, start_velocity, acceleration, delta_t, color=[0, 1, 1], num_segments=8):
#     assert len(start_position) == 3, "Input should be a numpy array of size 3"
#     assert len(start_velocity) == 3, "Input should be a numpy array of size 3"
#     #assert len(end_velocity) == 3, "Input should be a numpy array of size 3"
#     assert len(acceleration) == 3, "Input should be a numpy array of size 3"

#     # Calculate the time at each segment
#     times = np.linspace(0, delta_t, num_segments+1)
#     sim_dt = 
#     r = np.linspace(0, 1, num_segments+1)

#     # Calculate the position at each segment
#     positions = np.empty((num_segments+1, 3))
#     positions[0] = start_position
#     velocity = start_velocity
#     for i, t in enumerate(times):
#         if i == 0:
#             continue
#         velocity = velocity + acceleration * t
#         positions[i] = positions[i-1] + velocity * t + 0.5 * acceleration * t**2

#     # Draw the line segments
#     for i in range(num_segments):
#         p1 = positions[i]
#         p2 = positions[i+1]
#         print("drawing: ", p1, p2)
#         p.addUserDebugLine(p1.tolist(), p2.tolist(), [r[i],0,r[i]])
    
def drawPolynomial(start_position, start_velocity, acceleration, delta_t, color=[0, 1, 1], num_segments=100):
    assert len(start_position) == 3, "Input should be a numpy array of size 3"
    assert len(start_velocity) == 3, "Input should be a numpy array of size 3"
    assert len(acceleration) == 3, "Input should be a numpy array of size 3"

    # Calculate the time at each segment
    times = np.linspace(0, delta_t, num_segments+1)

    # Calculate the position at each segment
    positions = np.empty((num_segments+1, 3))
    for i, t in enumerate(times):
        positions[i] = start_position + start_velocity * t + 0.5 * acceleration * t**2

    # Draw the line segments
    for i in range(num_segments):
        p1 = positions[i]
        p2 = positions[i+1]
        #print("drawing: ", p1, p2)
        p.addUserDebugLine(p1.tolist(), p2.tolist(), color)

def write_json(new_data, filename='rrt_log.json'):
    if not os.path.isfile(filename):
        with open(filename,'w') as file:
            json.dump([new_data], file, indent=4)
    else:
        with open(filename,'r+') as file:
            file_data = json.load(file)
            file_data.append(new_data)
            # Sets file's current position at offset.
            file.seek(0)
            # convert back to json.
            json.dump(file_data, file, indent = 4)