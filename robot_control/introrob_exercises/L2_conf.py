# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os

dt = 0.001                                          # controller time step [s]
exp_duration = 5.0                                  # simulation duration
SLOW_FACTOR = 1                                     # to slow down simulation
frame_name = 'ee_link'                              # name of the frame to control (end-effector)

# PD controller
## Matrix of gains
kp = np.eye(6)*300                                  # proportional gains
kd = np.eye(6)*20                                   # derivative gains

## Parameters of the reference sinusoidal trajectory (1, 2, 3, 4, 5, 6)
exp_duration_sin = exp_duration - 1.0               # sine reference duration
amp = np.array([ 0.0, 0.2, 0.0, 0.0, 0.4, 0.0])     # amplitude
phi = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])     # phase
freq = np.array([ 0.0, 1.0, 0.0, 0.0, 1.5, 0.0])    # frequency

## EXERCISE 1.4: Bigger inertia variation on joint 2
#amp = np.array([ 0.0, 0.4, 0.8, 0.0, 0.4, 0.0])    # amplitude
#phi = np.array([ 0.0, 0.0, 3.14, 0.0, 0.0, 0.0])   # phase
#freq = np.array([ 0.0, 1.0, 1.0, 0.0, 1.5, 0.0])   # frequency

# Initial state
q0 = np.array([ 0.0, -0.3, 0.5, -1.57, -1.57, 0.5]) # joint configuration
qd0 = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])     # joint velocity
qdd0 = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])    # joint acceleration

#EXERCISE 1.4: high gains
#Kp = np.eye(6)*600
#kd = np.eye(6)*30

# EXERCISE 2.4: Add external force at T =2.0s
# Value of linear external force
extForce = np.array([0.0, 0.0, 50.0])
EXTERNAL_FORCE = False

# EXERCISE 2.7: Add (unilateral) compliant contact
n = np.array([0.0,0.0,1.0])                     # contact normal
p0 = np.array([0.0,0.0,0.0])                    # contact position
K_env = np.eye(3)*10000                         # contact stiffness
D_env = np.eye(3)*1000                          # contact damping
mu = 1.0                                        # friction coefficient (0.05)