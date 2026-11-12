# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np

dt = 0.001                                          # controller time step [s]
exp_duration = 5.0                                  # simulation duration
SLOW_FACTOR = 1                                     # to slow down simulation
frame_name = 'ee_link'                              # name of the frame to control (end-effector)

## Matrix of gains
# P linear gain
Kx= np.eye(3)
Kx[0,0] = 1000
Kx[1,1] = 1000
Kx[2,2] = 1000
# P angular gain
Dx = np.eye(3)
Dx[0,0] = 300
Dx[1,1] = 300
Dx[2,2] = 300

# P angular gain
Ko= np.eye(3)
Ko[0,0] = 800
Ko[1,1] = 800
Ko[2,2] = 800
# D angular gain
Do= np.eye(3)
Do[0,0] = 30
Do[1,1] = 30
Do[2,2] = 30

# Postural task
Kp_postural = 50.0
Kd_postural = 10.0

## Parameters of the reference Cartesian sinusoidal trajectory (1, 2, 3, 4, 5, 6)
exp_duration_sin = exp_duration - 1.0               # sine reference duration
amp= np.array([ 0.1, 0.0, 0.0])                     # amplitude
phi =np.array([ 0.0, 0.0, 0.0])                     # phase
freq=np.array([ 1.5, 0.0, 0.0])                     # frequency

# Initial configuration / velocity / Acceleration
q0  = np.array([ 0.0, -1, 1, 0.5, 0.4, 0.5])          # joint configuration
qd0 = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])     # joint velocity
qdd0 = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])    # joint acceleration

# EXE 2-3: Add external force
# Value of linear external force
extForce = np.array([0.0, 0.0, 200.0])
EXTERNAL_FORCE = True

RemoveInitialError = False