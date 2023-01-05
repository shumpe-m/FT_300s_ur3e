import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def command_pose(r = 0.54, theta = 0, primitive = "up"):
    x_bias = -0.1307
    z_bias = 0.06267

    position=np.zeros([3])
    
    position[0] = r * np.cos(theta + np.pi/2) + x_bias
    position[1] = r * np.sin(theta + np.pi/2)
    position[2] = 0.20 + z_bias if primitive == "up" else z_bias
    return position

file_name = "endeffector_pose_wall.npy"
#file_name = "endeffector_pose.npy"
data = np.load(file=file_name)

#print(data.shape)

if file_name == "endeffector_pose.npy":
    x_offset = 0.10997- (-0.33069) - 0.32
    y_offset = 0.33067 - -0.5
    z_offset = 0.93262 - 0.0626
    normalize = np.array([y_offset, x_offset, z_offset])

    roop_n = 7
    r_slope = 0.54 - 0.2
    data_pose = np.array([[0,0,0]])

    for theta_dx in range(5):
        for r_dx in range(roop_n):
            position = command_pose(r = 0.54 - r_slope/(roop_n - 1) * (r_dx), theta = np.pi/2 * theta_dx/4, primitive = "up")
            position += normalize
            data_pose = np.concatenate([data_pose, [position]], 0)
            position = command_pose(r = 0.54 - r_slope/(roop_n - 1) * (r_dx), theta = np.pi/2 * theta_dx/4, primitive = "down")
            position += normalize
            data_pose = np.concatenate([data_pose, [position]], 0)
else:
    x_offset = 0.12401631006 - (-0.470366524681)
    y_offset = 0.470357588006 - 0.0770377688695
    z_offset = 0.947901187812 - 0.014053577688
    normalize = np.array([y_offset*0, x_offset*0, z_offset])

    roop_n = 7
    r_slope = 0.6937 - 0.3472
    data_pose = np.array([[0,0,0]])

    for theta_dx in range(5):
        for r_dx in range(roop_n):
            position = command_pose(r = 0.6937 - r_slope/(roop_n - 1) * (r_dx), theta = np.pi/2 * theta_dx/4, primitive = "up")
            position += normalize
            data_pose = np.concatenate([data_pose, [position]], 0)
            position = command_pose(r = 0.6937 - r_slope/(roop_n - 1) * (r_dx), theta = np.pi/2 * theta_dx/4, primitive = "down")
            position += normalize
            data_pose = np.concatenate([data_pose, [position]], 0)


print(data_pose.shape)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.plot(data[:,0], data[:,1], data[:,2], color='blue')
#ax.scatter(data_pose[:,0], data_pose[:,1], data_pose[:,2], color='red')
ax.set_xlim(0, 0.6)
ax.set_ylim(0, 0.6)
ax.set_zlim(0.6, 1.2)
plt.show()

