
import sys

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)
from std_msgs.msg import Float64MultiArray
import math

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

CONTROLLERS = ["joint_group_pos_controller", "joint_group_vel_controller","scaled_pos_joint_traj_controller"]

class ContollerManager:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):
        rospy.init_node("test_move", anonymous=True)

        timeout = rospy.Duration(5)
        self.switch_controller_srv = rospy.ServiceProxy(
            "/ur5/controller_manager/switch_controller", SwitchController
        )
        self.load_controller_srv = rospy.ServiceProxy("/ur5/controller_manager/load_controller", LoadController)

        try:
            self.switch_controller_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.active_controller = CONTROLLERS[0]

        self.jointpub = rospy.Publisher('/ur5/'+CONTROLLERS[0]+'/command', Float64MultiArray, queue_size=10)




    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (CONTROLLERS)

        other_controllers.remove(target_controller)

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_controller_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_controller_srv(srv)


if __name__ == "__main__":
    cmanager = ContollerManager()

    cmanager.switch_controller(cmanager.active_controller)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        #msg.data = [0.0, -0.4, 0.5, -1.5, 0.0, -0.5]
        msg.data = [3.969757080078125 +0.6, -0.5977658194354554,  -2.526026487350464, -0.5901497763446351, -0.36137134233583623, 2.2153096199035645]

        #rospy.loginfo( msg.data)
        cmanager.jointpub.publish(msg)
        rate.sleep()
        if rospy.is_shutdown():
            rospy.loginfo("---- SHUTTING DOWN ----")