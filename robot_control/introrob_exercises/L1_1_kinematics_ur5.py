#common stuff 
from __future__ import print_function
import time as tm

from base_controllers.utils.common_functions import *
from base_controllers.utils.ros_publish import RosPub
from base_controllers.components.inverse_kinematics.inv_kinematics_pinocchio import robotKinematics
import L1_conf as conf

os.system("killall rosmaster rviz")
#instantiate graphic utils
ros_pub = RosPub("ur5")
robot = getRobotModel("ur5")
kin = robotKinematics(robot, conf.frame_name)

##############################
##exercise 2.6 : postural task
###############################
## desired task space position
p = np.array([0.5, -0.2, 0.5])
# initial guess
q_i = np.array([0.0, -0.0, 0.0, 0.0, 0., 0.0])

# postural elbow down
q_postural = np.array([0.0, 0.8, -0.8, -0.8, -0.8, 0.0])
# postural elbow up
#q_postural = np.array([0.0, -1.8, 1.8, -0.8, -0.8, 0.0])

q_ik, _, _ = kin.endeffectorInverseKinematicsLineSearch(p, conf.frame_name,
                                                        q_i,
                                                        verbose = True,
                                                        use_error_as_termination_criteria = False,
                                                        postural_task = True,
                                                        w_postural = 0.001,
                                                        q_postural = q_postural)
print("Desired End effector \n", p)

robot.computeAllTerms(q_ik, np.zeros(6))
p_ik = robot.framePlacement(q_ik, robot.model.getFrameId(conf.frame_name)).translation
task_diff = p_ik - p
print("Point obtained with IK solution \n", p_ik)
print("Error at the end-effector: \n", np.linalg.norm(task_diff))
print("Final joint positions\n", q_ik)

tm.sleep(2.)
ros_pub.add_marker(p)
ros_pub.publish(robot, q_i)
tm.sleep(5.)

ros_pub.add_marker(p)
ros_pub.publish(robot, q_postural)
tm.sleep(5.)


ros_pub.add_marker(p)
ros_pub.publish(robot, q_ik)
tm.sleep(30.)
ros_pub.deregister_node()





