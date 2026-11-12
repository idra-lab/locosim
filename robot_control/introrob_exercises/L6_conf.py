# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os

LINE_WIDTH = 60

dt = 0.004  # controller time step
exp_duration = 5.0 #simulation duration
CONTINUOUS = False
verbose = False

# Initial configuration / velocity / Acceleration
q0  = np.matrix([ 0.0, -0.6, 0.6, -1.67, -1.57, 0.0]).T
qd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
qdd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T

# Parameters of Joint Reference Trajectories (X,Y, Z, Roll, Pitch, Yaw), (X,Y we assume them in the horizontal frame (as base frame but aligned with gravity)
amp                  = np.array([ 0.0, 0.0, 0.03, 0.0, 0.1, 0.0]).T     # amplitude
freq                 = np.array([ 0.0, 0.0, 0.5, 0.0, 1.0, 0.0]).T           # frequency (time 2 PI)
phi                  = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T     # phase

buffer_size = 30001

# Gains for the virtual model
control_params = {}
control_params['hyq'] = {'Kp_lin_x': 2000, 'Kp_lin_y': 2000, 'Kp_lin_z': 2000,    
                       'Kd_lin_x': 200, 'Kd_lin_y': 200, 'Kd_lin_z': 200,                       
                       'KpRoll': 1000, 'KpPitch': 1000, 'KpYaw': 1000, 
                       'KdRoll': 100, 'KdPitch': 100, 'KdYaw': 100, 'gravity': 9.81}


control_params['solo'] = {'Kp_lin_x': 100, 'Kp_lin_y': 100, 'Kp_lin_z': 100,
                       'Kd_lin_x': 10, 'Kd_lin_y': 10, 'Kd_lin_z': 10,
                       'KpRoll': 100, 'KpPitch': 100, 'KpYaw': 100,
                       'KdRoll': 1, 'KdPitch': 1, 'KdYaw': 1, 'gravity': 9.81}

control_params['go1'] = {'Kp_lin_x': 300, 'Kp_lin_y': 300, 'Kp_lin_z': 800,
                       'Kd_lin_x': 50, 'Kd_lin_y': 50, 'Kd_lin_z': 50,
                       'KpRoll': 50, 'KpPitch': 50, 'KpYaw': 50,
                       'KdRoll': 3, 'KdPitch': 3, 'KdYaw': 3, 'gravity': 9.81}

control_params['aliengo'] = {'Kp_lin_x': 500, 'Kp_lin_y': 500, 'Kp_lin_z': 500,
                       'Kd_lin_x': 100, 'Kd_lin_y': 100, 'Kd_lin_z': 100,
                       'KpRoll': 200, 'KpPitch': 200, 'KpYaw': 200,
                       'KdRoll': 20, 'KdPitch': 20, 'KdYaw': 20, 'gravity': 9.81}

   