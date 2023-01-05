#!/usr/bin/env python
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class Data_plot:
    def __init__(self):
        self.robot_spawn_point = [0.11, 0.0, 0.87]
        fig = plt.figure()
        self.ax = fig.gca(projection='3d')
        # self.ax.set_xlim(0, 0.6)
        # self.ax.set_ylim(0, 0.6)
        # self.ax.set_zlim(0, 0.6)

    def plot_data(self, data_pose):
        #self.ax.scatter(data_pose[:,0], data_pose[:,1], data_pose[:,2], color='green')
        self.ax.plot(data_pose[:,0], data_pose[:,1], data_pose[:,2], color = "red")

    def data_load(self):
        file_name = "leftfinger_position.npy"
        print(file_name)
        data = np.load(file=file_name)
        z_idx = np.where(data[:,2] != 0)
        data = data[z_idx]
        return data

def main():
    plotting = Data_plot()
    data = plotting.data_load()
    print(data.shape)
    plotting.plot_data(data_pose = data)
    


    # for h_dx in range(6):
    #     for r_dx in range(6):
    #         TF = plotting.data_load(data_num=2)
    #         first_TF = TF[l_num ,0]
    #         count = plotting.success_count(TF)
    #         data_goal = plotting.data_load(data_num=0)
    #         plotting.plot_data(data_pose = data_goal, count = count, TF = first_TF)
    #         r_scale += r_dx * 0.2
    #         plotting.set_value(r_scale, height)
    #     r_scale = 0.0
    #     height += 0.05 * h_dx
    #     plotting.set_value(r_scale, height)

    plt.show()




if __name__ == '__main__':
    main()