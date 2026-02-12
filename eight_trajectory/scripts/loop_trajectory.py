#!/usr/bin/env python3

import numpy as np
from plot_functions import (
        plot_trajectory, 
        plot_error)

path_to_file = '/home/user/ros2_ws/src/checkpoint16/eight_trajectory/scripts'
open_loop_trajectory = np.load(path_to_file+'trajectory.npy')
waypoints = np.load(path_to_file+'waypoints.npy')

# ref_points = []

plot_trajectory(open_loop_trajectory, waypoints)
# plot_error(waypoints, ref_points)
