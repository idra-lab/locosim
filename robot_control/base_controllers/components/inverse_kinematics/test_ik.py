from __future__ import print_function
import numpy as np
from  inv_kinematics_pinocchio import robotKinematics
import time


import sys
sys.path.append('../utils')#allows to incude stuff on the same level
from base_controllers.utils.common_functions import getRobotModel

np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)


robot_name = "solo"

LF_foot = np.array([0.19683,  0.19107, -0.21159])
RF_foot = np.array([0.19678, -0.19122, -0.21158])
LH_foot = np.array([-0.19679,  0.19115, -0.21158])
RH_foot = np.array([-0.19681, -0.19111, -0.21116])

#use a reasonable guess
q0 = np.vstack((np.array([0.0, 0.5, -1.5]),
               np.array([-0.0, 0.5, -1.5]),
              np.array([-0.0, -0.5, 1.5]),
              np.array([-0.0, -0.5, 1.5])))
              
#robot_name = "hyq"
#LF_foot = np.array([  0.33038,  0.31269, -0.60686])
#RF_foot = np.array([ 0.33034, -0.31262, -0.60685])
#LH_foot = np.array([-0.33283,  0.30606, -0.60562])
#RH_foot = np.array([-0.33333, -0.30623, -0.60561])
#
##use a reasonable guess
#q0 = np.vstack((np.array([-0.2, 0.7, -1.4]),
#               np.array([-0.2, 0.7, -1.4]),
#              np.array([-0.2, -0.7, 1.4]),
#              np.array([-0.2, -0.7, 1.4])))
              
              
#robot_name = 'aliengo'
#LF_foot = np.array([0.2331, 0.10884, -0.19765])
#RF_foot = np.array([0.2331, -0.10884, -0.1976])
#LH_foot = np.array([-0.2183, 0.10884, -0.1976])
#RH_foot = np.array([-0.2183, -0.10884, -0.1976])
#
#
##use a reasonable guess 
#q0 = np.vstack((np.array([-0.2, 0.75, -1.5]), 
#               np.array([-0.2, 0.75, -1.5]), 
#              np.array([-0.2, 0.75, -1.5]),
#              np.array([-0.2, 0.75, -1.5])))                
              
            
robot = getRobotModel(robot_name)
ee_frames = ['lf_foot', 'rf_foot', 'lh_foot','rh_foot']

# feet_pos_des = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot, LC_foot, RC_foot))
feet_pos_des = np.vstack((LF_foot, RF_foot, LH_foot, RH_foot))
#print ("feet_pos_des: ", feet_pos_des)

kin = robotKinematics(robot, ee_frames)
start_time = time.time()
q = kin.leggedRobotInverseKinematics(feet_pos_des, q0.ravel(), verbose = True)
print('total time is ',time.time()-start_time)
#print('q is:\n', q)
print('\n flatten result is: \n%s' % q.flatten().tolist())

