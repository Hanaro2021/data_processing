import pandas as pd
import matplotlib.pyplot as plt
import config
import data
from trajectories import IMUTrajectory, GPSTrajectory, SimpleTrajectory
import os

file_path = "./input_data/identity3-B_lora_data.csv"
fig_size = (10, 10)

if __name__ == '__main__':
    obj = data.data_format(file_path)
    obj.set_initial_condition()
    obj.ekf()

    # obj.plot_data(
    #     obj.time[config.index["launch"]:config.index["touchdown"]],
    #     obj.altitude[config.index["launch"]:config.index["touchdown"]],
    # )

    fig = plt.figure(figsize=fig_size)
    ax = fig.add_subplot(111, projection='3d')
    obj.plot_kalman_trajectory(ax, color="red", label="Kalman")
    obj.plot_gps_trajectory(ax, color="green", label="GPS")
    obj.plot_simple_trajectory(ax, color="blue", label="Simple")
    obj.print_time()

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_xlim(-100, 100)
    ax.set_ylim(-100, 100)
    ax.set_zlim(0, 300)
    ax.legend()

    plt.show()
