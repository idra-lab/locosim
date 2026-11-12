from __future__ import print_function
import numpy as np
from  inv_kinematics_pinocchio import robotKinematics
import time
import rospkg
'''
Unit test for Ur5 fixed base robot inverse kinematics, considering as input only the position end effector  (need to solve for redundancy) 
'''

import sys
sys.path.append('../utils')#allows to incude stuff on the same level
from base_controllers.utils.common_functions import getRobotModel
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
from termcolor import colored

#use a reasonable guess
q_guess = np.array([0.55, -0.7, 1.0, -1.57, -1.57, 0.5])
robot_name = "ur5"
xacro_path = rospkg.RosPack().get_path('ur_description') + '/urdf/ur5.urdf.xacro'
robot = getRobotModel(robot_name, generate_urdf=True, xacro_path=xacro_path)
ee_frame = 'tool0'
use_postural_task = True # without the postural task the solution depends on the initial guess!!

## IMPORTANT these value is reasonable only for ur_description urdf ! not for the one in example robot data! they have base frames rotated!
ee_pos_des = np.array([-0.52636,  0.48073, -0.491 ])
kin = robotKinematics(robot, ee_frame)
start_time = time.time()

q_postural = np.array([0.8, -0.8, 0.8, -0.8, -0.8, 0.8])
q, ik_success, out_of_workspace = kin.endeffectorInverseKinematicsLineSearch(ee_pos_des,ee_frame, 
                                                                             q_guess, 
                                                                               verbose = True, 
                                                                               use_error_as_termination_criteria = False, 
                                                                               postural_task = use_postural_task,
                                                                               w_postural = 0.0001,
                                                                               q_postural = q_postural)
print('total time is ',time.time()-start_time)
#print('q is:\n', q)
print('Result is: ' , q)

if (use_postural_task):
    qtest = np.array([ 0.32509, -0.9816,   0.58257, -0.90407, -0.79274,  0.8    ])
else:
    qtest = np.array([ 0.55427, -1.15469,  0.83492, -1.53084, -1.57799,  0.5    ])

#last joint does not affect end effector position (only orientation) so I will discard from test
if np.allclose(q[:-1], qtest[:-1], 0.001):
    print(colored("TEST PASSED", "green"))
else:
    print(colored("TEST NOT PASSED", "red"))
