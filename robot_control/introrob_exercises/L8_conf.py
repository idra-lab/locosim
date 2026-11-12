# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""
import numpy as np

# EXE L8-1.2: Sinisoidal joint reference
amplitude = 0.1
frequency = 0.25

# EXE L8-1.3: Constant Cartesian reference
ee_reference = np.array([-0.3, 0.5, -0.5])

# EXE L8-1.4: Desired orientation
des_orient = np.array([ -1.9, -0.5, -0.1])

# EXE L8-2.1: admittance control
admittance_control = False

# EXE L8-2.5: obstacle avoidance
obstacle_avoidance = False
des_ee_goal = np.array([0.7, 1.1, 1.2])

# EXE L8-5:  polynomial trajectory
poly_duration = 3.0

# EXE L2-2.3: admittance control
Kx = 600 * np.identity(3)
Dx = 200 * np.identity(3)


# Del Prete Trajectory tracking
USER_TRAJECTORY = False

import os
# list of trajectories file names to track in sequence
traj_folder = os.environ["LOCOSIM_DIR"]+"/robot_control/lab_exercises/custom_traj/"
traj_file_name = ['home_2_table_robot', 'table_2_belt_robot', 'belt_2_home_robot']
# traj_file_name = 'home_2_table_2_belt_2_home_robot'
traj_slow_down_factor = 1 # how many times the reference trajectory has to be slowed down
