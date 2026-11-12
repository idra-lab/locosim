# Description
# File contains some necessary control algorithms for HyQ
# Author: Michele Focchi
# Date: 04-12-2022
import rospkg
import numpy as np
import rospy as ros
from base_controllers.components.gripper_manager import GripperManager
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from termcolor import colored

# controller manager management
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController


class ControllerManager():
    def __init__(self, robot_name, conf):
        self.robot_name = robot_name
        self.conf = conf
        self.control_type = conf['control_type']
        self.gripper_sim = conf['gripper_sim']
        self.gripper_type = conf['gripper_type']
        self.real_robot = conf['real_robot']
        self.number_of_joints = len(conf['joint_names'])

        if  (self.control_type == 'torque'):
            print(colored("Controller Manager: torque", "blue"))
        if (self.control_type == 'position'):
            print(colored("Controller Manager: position", "blue"))

    def initPublishers(self, robot_name):
        # publisher for ros_impedance_controller
        self.pub_full_jstate = ros.Publisher("/command", JointState, queue_size=1, tcp_nodelay=True)
        # specific publisher for joint_group_pos_controller that publishes only position
        self.pub_reduced_des_jstate = ros.Publisher("/" + robot_name + "/joint_group_pos_controller/command",
                                                    Float64MultiArray, queue_size=10)

        self.switch_controller_srv = ros.ServiceProxy(
            "/" + self.robot_name + "/controller_manager/switch_controller", SwitchController)
        self.load_controller_srv = ros.ServiceProxy("/" + self.robot_name + "/controller_manager/load_controller",
                                                    LoadController)

        #  different controllers are available from the real robot and in simulation in case of position control
        if self.real_robot:
            self.available_controllers = [
                "joint_group_pos_controller",
                "scaled_pos_joint_traj_controller"]
        else:
            self.available_controllers = ["joint_group_pos_controller",
                                          "pos_joint_traj_controller"]
        self.active_controller = self.available_controllers[0]

        # switch to the selected controller
        if (self.conf['control_mode'] == "trajectory"):
            if (self.real_robot):
                self.switch_controller("scaled_pos_joint_traj_controller")
            else:
                self.switch_controller("pos_joint_traj_controller")
        else: # control_mode point
            if self.control_type == 'position':
                self.switch_controller("joint_group_pos_controller")

        # instantiate the gripper manager that will read soft gripper param from param server
        self.gm = GripperManager(self.gripper_type, self.real_robot, self.conf['dt'])

    def send_full_jstate(self, q_des, qd_des, tau_ffwd):
         # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
         msg = JointState()
         if self.gripper_sim:
             msg.position = np.append(q_des, self.gm.getDesGripperJoints())
             msg.velocity = np.append(qd_des, np.zeros(self.gm.number_of_fingers))
             msg.effort = np.append(tau_ffwd,  np.zeros(self.gm.number_of_fingers))
         else:
             msg.position = q_des
             msg.velocity = qd_des
             msg.effort = tau_ffwd
         self.pub_full_jstate.publish(msg)

    def send_reduced_des_jstate(self, q_des):
        msg = Float64MultiArray()
        if  self.gripper_sim and not self.real_robot:
            msg.data = np.append(q_des, self.gm.getDesGripperJoints())
        else:
            msg.data = q_des
        self.pub_reduced_des_jstate.publish(msg)

    def sendReference(self, q_des, qd_des = None, tau_ffwd = None):
        if (self.control_type == 'torque'):
            if qd_des is None:
                qd_des = np.zeros(self.number_of_joints)
            if tau_ffwd is None:
                tau_ffwd = np.zeros(self.number_of_joints)
            self.send_full_jstate(q_des, qd_des, tau_ffwd)
        else:
            self.send_reduced_des_jstate(q_des)


    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        print('Available controllers: ', self.available_controllers)
        print('Controller manager: loading ', target_controller)

        other_controllers = (self.available_controllers)
        other_controllers.remove(target_controller)
        print('Controller manager:Switching off  :  ', other_controllers)

        srv = LoadControllerRequest()
        srv.name = target_controller

        self.load_controller_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_controller_srv(srv)
        self.active_controller = target_controller



