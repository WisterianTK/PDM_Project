import pybullet as p

def drawPoint(point, color, size=0.1):
    assert len(point) == 3, "Input should be a numpy array of size 3"
    x, y, z = point[0], point[1], point[2]
    #size = 0.1  # Size of the cross
    #color = [0, 0, 0]  # RGB color
    p.addUserDebugLine([x-size, y, z], [x+size, y, z], color)
    p.addUserDebugLine([x, y-size, z], [x, y+size, z], color)
    p.addUserDebugLine([x, y, z-size], [x, y, z+size], color)
