from __future__ import print_function
import numpy as np
from  inv_kinematics_pinocchio import robotKinematics
import time
import rospkg

import sys
sys.path.append('../utils')#allows to incude stuff on the same level
from base_controllers.utils.common_functions import getRobotModel
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
from termcolor import colored
from base_controllers.utils.math_tools import Math

'''
Unit test for Ur5 fixed base robot inverse kinematics, considering as inputs the position end effector and its orientation
'''

math_utils = Math()
#if you use a reasonable guess it will converge quicker
q_guess = np.array([0.4, -1.1,  1.0, -6.,  1,  1.0])
#q_guess = np.zeros(6)
robot_name = "ur5"
xacro_path = rospkg.RosPack().get_path('ur_description') + '/urdf/ur5.urdf.xacro'
robot = getRobotModel(robot_name, generate_urdf=True, xacro_path=xacro_path)

ee_frame = 'tool0'

## IMPORTANT these value is reasonable only for ur_description urdf ! not for the one in example robot data! they have base frames rotated!
ee_pos_des = np.array([-0.52636,  0.48073, -0.491 ])
#This is the result it should get close 
qtest = np.array([ 0.76689, -1.17447,  1.08174, -6.18935,  2.33769,  1.57316])

#orient reference
rpy_des = np.array([0, -1.57 , 0])
# compute rotation matrix representing the desired orientation from Euler Angles
w_R_e_des = math_utils.eul2Rot(rpy_des)   
print(w_R_e_des)
kin = robotKinematics(robot, ee_frame)
start_time = time.time()
q, ik_success, out_of_workspace = kin.endeffectorFrameInverseKinematicsLineSearch(ee_pos_des, w_R_e_des, ee_frame, q_guess, verbose = True, wrap = True)

#with postural
q_postural = np.array([0.8, -0.8, 0.8, -0.8, -0.8, 0.8])
q, ik_success, out_of_workspace = kin.endeffectorFrameInverseKinematicsLineSearch(ee_pos_des, w_R_e_des, ee_frame, q_guess, verbose = True, wrap = True,
                                                                                  postural_task=True,
                                                                                  w_postural=0.0001,
                                                                                  q_postural=q_postural )

#chek orient
print( robot.framePlacement(q, robot.model.getFrameId(ee_frame)).rotation)

print('total time is ',time.time()-start_time)
#print('q is:\n', q)
print('Result is: ' , q)
#last joint does not affect end effector position (only orientation) so I will discard from test
if np.allclose(q[:-1], qtest[:-1], 0.001):
    print(colored("TEST PASSED", "green"))


