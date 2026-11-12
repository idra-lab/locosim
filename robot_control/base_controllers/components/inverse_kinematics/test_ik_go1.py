import pinocchio as pin
import numpy as np
from base_controllers.utils.custom_robot_wrapper import RobotWrapper
from base_controllers.utils.common_functions import getRobotModelFloating
import os

from base_controllers.components.inverse_kinematics.inv_kinematics_quadruped import InverseKinematics
# [array([ 0.16048,  0.17749, -0.27398]), LF
#  array([-0.2158 ,  0.17761, -0.27397]), LH
#  array([ 0.16048, -0.1775 , -0.27398]), RF
#  array([-0.21582, -0.17764, -0.27396])] RH

robot = getRobotModelFloating("go1")

legs = ['lf', 'lh', 'rf', 'rh']
hips = ['HipDown'] * 4
knees = ['']*4
knees[0] = 'KneeInward' #LF
knees[1] = 'KneeOutward' #LH
knees[2] = 'KneeInward' #RF
knees[3] = 'KneeOutward' #RH

#Note the order of legs matters!
q = np.array([ 0.17855,  0.90267, -1.61727,
               0.17895,  0.90279, -1.61698,
               -0.17859,  0.90266,-1.61727,
               -0.17909,  0.9028 , -1.61689])

qf = pin.neutral(robot.model)
qf[7:12 + 7] = q

pin.forwardKinematics(robot.model, robot.data, qf)
pin.updateFramePlacements(robot.model, robot.data)
feet_id = [robot.model.getFrameId(leg + '_foot') for leg in legs]
feet_pos = [robot.data.oMf[foot].translation.copy() for foot in feet_id]

print(feet_pos[0])
print(feet_pos[1])
print(feet_pos[2])
print(feet_pos[3])

# #should be
# [ 0.16048  0.17749 -0.27398]
# [ 0.16048 -0.1775  -0.27398]
# [-0.2158   0.1776  -0.27397]
# [-0.21582 -0.17765 -0.27396]


IK = InverseKinematics(robot)

for i, foot in enumerate(feet_id):
    print('EE name:', robot.model.frames[foot].name)
    print('\tPosition:', feet_pos[i])

    sol, isFeasible = IK.ik_leg(feet_pos[i], legs[i], hips[i], knees[i])

    print('\tIK Solution Analytics:', sol)

