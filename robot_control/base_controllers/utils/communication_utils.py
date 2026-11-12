import numpy as np
from termcolor import colored
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion
import rospy as ros

def getInitialStateFromOdom(robot_name = None):
    try:
        odom0 = ros.wait_for_message("/" + robot_name + "/odom", Odometry, timeout=10.0)
    except ros.ROSException:
        ros.logerr(f"Timed out waiting for /{robot_name}/odom")
        return

    p0 = odom0.pose.pose.position
    q0 = odom0.pose.pose.orientation
    yaw0 = euler_from_quaternion([q0.x, q0.y, q0.z, q0.w])[2]
    print(colored(f"{robot_name}: Init. desired state from first /odom: x0: {p0.x},y0: {p0.y},yaw0: {yaw0}", "red"))

    return p0.x, p0.y, yaw0

def getInitialStateFromJoints(robot_name = None, joint_names = None):
    try:
        msg = ros.wait_for_message("/" + robot_name + "/joint_states", JointState, timeout=10.0)
    except ros.ROSException:
        ros.logerr(f"Timed out waiting for /{robot_name}/odom")
        return
    n_joints = len(msg.name)
    q0 = np.zeros(n_joints)
    for msg_idx in range(n_joints):
        for joint_idx in range(len(joint_names)):
            if joint_names[joint_idx] == msg.name[msg_idx]:
                q0[joint_idx] = msg.position[msg_idx]
    print(colored(f"{robot_name}: Init. desired joints state: q0: {q0}", "red"))
    return q0