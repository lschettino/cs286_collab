import matplotlib.pyplot as plt 
import pandas as pd 

task = "task3"
poses = pd.read_csv(task + "_poses.txt", header=None,  sep = " ") 
poses_odom = pd.read_csv(task + "_poses_odom.txt", header = None, sep = " ") 

x_poses = poses.iloc[:, 3].to_list()
y_poses = poses.iloc[:, 7].to_list()

x_poses_odom = poses_odom.iloc[:, 3].to_list()
y_poses_odom = poses_odom.iloc[:, 7].to_list()




plt.plot(x_poses, y_poses, label = "Map graph")
plt.plot(x_poses_odom, y_poses_odom, label = "Odometry")

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Plot showing overlayed trajectories") 
plt.legend()
plt.savefig(task + '_2d_plot_overlayed.png')
