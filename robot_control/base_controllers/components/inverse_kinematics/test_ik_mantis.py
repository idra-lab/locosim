import pinocchio as pin
import numpy as np
from base_controllers.utils.custom_robot_wrapper import RobotWrapper
from base_controllers.utils.common_functions import getRobotModelFloating
from  base_controllers.components.inverse_kinematics.inv_kinematics_pinocchio import robotKinematics
import os

ee_frames = ['lf_foot', 'lh_foot', 'rf_foot','rh_foot']
q0 = np.vstack((np.array([0.0, 0.5, -1.5]),
               np.array([-0.0, 0.5, -1.5]),
              np.array([-0.0, 0.5, -1.5]),
              np.array([-0.0, 0.5, -1.5])))


#with locosim one (need to run once thesimulation) hips are not always negative outwards!
# robot = getRobotModelFloating("mantis")
#
# #Note the order of legs matters!
# q = np.array([ 0.1,  0.8, -1.6,#LF
#                0.1,  0.8, -1.6,#LH
#                -0.1,  0.8,-1.6,#RF
#                -0.1,  0.8 , -1.6])#RH

#with the mantis_ros_static wals framework one hips are  always negative outwards!
path = os.environ.get('LOCOSIM_DIR')
urdf_location = path + "/robot_urdf/mantis_framework.urdf"
robot = RobotWrapper.BuildFromURDF(urdf_location, root_joint=pin.JointModelFreeFlyer())

#Note the order of legs matters!
q = np.array([ -0.1,  0.8, -1.6,#LF //outwards
               -0.1,  0.8, -1.6,#LH //outwards
               -0.1,  0.8,-1.6,#RF //outwards
               -0.1,  0.8 , -1.6])#RH //outwards

qf = pin.neutral(robot.model)
qf[7:12 + 7] = q

pin.forwardKinematics(robot.model, robot.data, qf)
pin.updateFramePlacements(robot.model, robot.data)
feet_id = [robot.model.getFrameId(i) for i in ee_frames]
feet_pos = [robot.data.oMf[foot].translation.copy() for foot in feet_id]
feet_pos_des = np.vstack((feet_pos[0], feet_pos[1], feet_pos[2], feet_pos[3]))
print(feet_pos[0])
print(feet_pos[1])
print(feet_pos[2])
print(feet_pos[3])

kin = robotKinematics(robot, ee_frames)
q = kin.leggedRobotInverseKinematics(feet_pos_des, q0.ravel(), verbose = True)
print('q is:\n', q)

