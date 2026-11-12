# -*- coding: utf-8 -*-
"""
Created on 3 May  2022

@author: mfocchi
"""

from __future__ import print_function
import os
import rospy as ros
import threading

# messages for topic subscribers
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyRequest
from termcolor import colored

#gazebo messages
from gazebo_msgs.srv import SetModelState
#gazebo services
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsPropertiesRequest
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest

# ros utils
import roslaunch
import rospkg

#other utils
from base_controllers.utils.ros_publish import RosPub
from base_controllers.utils.pidManager import PidManager
from base_controllers.utils.utils import Utils
from base_controllers.utils.math_tools import *
from numpy import nan
from base_controllers.utils.common_functions import getRobotModel
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
from termcolor import colored
import matplotlib.pyplot as plt
import distro
import rosgraph
import rosnode
import sys

import  base_controllers.params as conf
# robots can be ur5 and jumpleg to load ur5 you need to set this xacro path in loadModelAndPublishers
robotName = "ur5"

from base_controllers.components.inverse_kinematics.inv_kinematics_pinocchio import robotKinematics
from base_controllers.utils.math_tools import Math
from gazebo_msgs.srv import ApplyBodyWrench
from base_controllers.utils.common_functions import plotJoint, checkRosControllerRunning

class BaseControllerFixed(threading.Thread):
    """
        This Class can be used to simulate floating base robots that
        have an under-actuated base (e.g. like quadrupeds, mobile robots)

        ...

        Attributes
        ----------
        q, q_des : numpy array
           actual /desired joint positions
        qd, qd_des : numpy array
            actual /desired joint velocities
        tau, tau_ffwd :  numpy array
            actual /feed-forward torques
        x_ee  : numpy array
            position of the end-effector expressed in the base_link frame
        contactForceW : numpy array
            contact force at the end-effector expressed in the world frame
        contactMomentW : numpy array
            contact moment at the end-effector expressed in the world frame

        Methods
        -------
        loadModelAndPublishers(xacro_path=None)
           loads publishers for visual features (ros_pub), joint commands and declares subscriber to /ground_truth and /joint_states
        startSimulator(world_name = None)
           Starts gazebo simulator with ros_impedance_controller
        send_des_jstate(self, q_des, qd_des, tau_ffwd)
            publishes /command topic with set-points for joint positions, velocities and feed-forward torques
        startupProcedure():
            initialize PD gains
        initVars()
            initializes class variables
        logData()
            fill in the X_log variables for plotting purposes, it needs to be called at every loop
        _receive_jstate(msg)
            callback associated to the joint state subscriber, fills in q, qd, tau arrays

    """
    def __init__(self, robot_name="ur5", external_conf = None):
        threading.Thread.__init__(self)

        if (external_conf is not None):
            conf.robot_params = external_conf.robot_params

        self.robot_name = robot_name

        self.base_offset = np.array([ conf.robot_params[self.robot_name]['spawn_x'],
                                      conf.robot_params[self.robot_name]['spawn_y'],
                                     conf.robot_params[self.robot_name]['spawn_z']])
        self.u = Utils()
        self.math_utils = Math()
        self.contact_flag = False
        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        #send data to param server
        self.verbose = conf.verbose
        self.use_torque_control = True

        print("Initialized fixed basecontroller---------------------------------------------------------------")

    def startSimulator(self, world_name = None,  additional_args = None, launch_file = None):
        os.system("pkill rosmaster")
        os.system("pkill gzserver")
        os.system("pkill gzclient")
        os.system("pkill rviz")
        if launch_file=='standard':
            launch_file = rospkg.RosPack().get_path('ros_impedance_controller') + '/launch/start_framework.launch'

        if launch_file is None:
            launch_file = rospkg.RosPack().get_path('ros_impedance_controller') + '/launch/ros_impedance_controller_' + self.robot_name + '.launch'


        # clean up previous process
        os.system("killall rosmaster rviz gzserver gzclient")
        if (distro.linux_distribution()[1] == "16.04"):
            print(colored("This file only works with distribution from ROS lunar (I.e. ubuntu 17.04 or later) ", "red"))
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [launch_file,
                    'robot_name:=' + self.robot_name,
                    'spawn_x:=' + str(conf.robot_params[self.robot_name]['spawn_x']),
                    'spawn_y:=' + str(conf.robot_params[self.robot_name]['spawn_y']),
                    'spawn_z:=' + str(conf.robot_params[self.robot_name]['spawn_z'])]
        cli_args.append('use_torque_control:=' + str(self.use_torque_control).lower())
        if additional_args is not None:
            cli_args.extend(additional_args)
        if world_name is not None:
            print(colored("Setting custom model: "+str(world_name), "blue"))
            cli_args.append('world_name:=' + str(world_name))

        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        ros.sleep(1.0)
        print(colored('SIMULATION Started', 'blue'))

    def loadModelAndPublishers(self,  xacro_path = None, additional_urdf_args = None, markers_time_to_live = 0.):

        # instantiating objects
        self.ros_pub = RosPub(self.robot_name, only_visual=True, markers_time_to_live=markers_time_to_live)
        self.pub_des_jstate = ros.Publisher("/command", JointState, queue_size=1, tcp_nodelay=True)
        # freeze base  and pause simulation service
        self.reset_world = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_physics_client = ros.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.get_physics_client = ros.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        self.pause_physics_client = ros.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = ros.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.reset_joints_client = ros.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        self.u.putIntoGlobalParamServer("verbose", self.verbose)
        # subscribers
        self.sub_jstate = ros.Subscriber("/" + self.robot_name + "/joint_states", JointState,
                                         callback=self._receive_jstate, queue_size=1,buff_size = 2 ** 24, tcp_nodelay=True)

        self.apply_body_wrench = ros.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

        if (self.use_torque_control):
            self.pid = PidManager(self.joint_names)

        # Loading a robot model of robot (Pinocchio)
        if xacro_path is None:
            print(colored(f"setting default xacro path", "blue"))
            xacro_path = rospkg.RosPack().get_path(
                self.robot_name + '_description') + '/urdf/' + self.robot_name + '.xacro'
        else:
            print(colored(f"loading custom xacro path:  : {xacro_path}", "blue"))
        self.robot = getRobotModel(self.robot_name, generate_urdf=True, xacro_path=xacro_path,
                                   additional_urdf_args=additional_urdf_args)

    def _receive_jstate(self, msg):
         for msg_idx in range(len(msg.name)):          
             for joint_idx in range(len(self.joint_names)):
                 if self.joint_names[joint_idx] == msg.name[msg_idx]: 
                     self.q[joint_idx] = msg.position[msg_idx]
                     self.qd[joint_idx] = msg.velocity[msg_idx]
                     self.tau[joint_idx] = msg.effort[msg_idx]


    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
         # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
         msg = JointState()
         msg.position = q_des
         msg.velocity = qd_des
         msg.effort = tau_ffwd                
         self.pub_des_jstate.publish(msg)  
                
    def deregister_node(self):
        print( "deregistering nodes"     )
        self.ros_pub.deregister_node()


    def startupProcedure(self):
        if (self.use_torque_control):
            if not checkRosControllerRunning("ros_impedance_controller", self.robot_name):
                print(colored('Error: you need to launch the ros impedance controller in torque mode!', 'red'))
                sys.exit()
            self.pid.setPDjoints( conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'], np.zeros(self.robot.na))
        print(colored("Startup accomplished -----------------------","red"))

    def initVars(self):

        self.q = np.zeros(self.robot.na)
        self.qd = np.zeros(self.robot.na)
        self.tau = np.zeros(self.robot.na)
        self.q_des =np.zeros(self.robot.na)
        self.qd_des = np.zeros(self.robot.na)
        self.tau_ffwd =np.zeros(self.robot.na)

        self.x_ee = np.zeros(3)
        self.x_ee_des = np.zeros(3)

        self.contactForceW = np.zeros(3)
        self.contactMomentW = np.zeros(3)

        self.time  = 0.

        #log vars
        self.q_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.q_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] )) *nan
        self.qd_des_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.qd_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] )) *nan
        self.tau_ffwd_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.tau_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] ))*nan

        self.x_ee_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.x_ee_des_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.contactForceW_log = np.empty((3,conf.robot_params[self.robot_name]['buffer_size'] ))  *nan
        self.time_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))*nan

        self.log_counter = 0

        self.ikin = robotKinematics(self.robot, conf.robot_params[self.robot_name]['ee_frame'])


    def logData(self):
        if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
            self.q_des_log[:, self.log_counter] = self.q_des
            self.q_log[:,self.log_counter] =  self.q
            self.qd_des_log[:,self.log_counter] =  self.qd_des
            self.qd_log[:,self.log_counter] = self.qd
            self.tau_ffwd_log[:,self.log_counter] = self.tau_ffwd                    
            self.tau_log[:,self.log_counter] = self.tau
            self.x_ee_log[:, self.log_counter] = self.x_ee
            self.x_ee_des_log[:, self.log_counter] = self.x_ee_des
            self.contactForceW_log[:,self.log_counter] =  self.contactForceW
            self.time_log[self.log_counter] = self.time
            self.log_counter+=1
  # if self.log_counter > 0 and (self.log_counter % conf.robot_params[self.robot_name]['buffer_size']) == 0:
  #       self.q_des_log = self.log_policy(self.q_des_log)
  #       self.q_log = self.log_policy(self.q_log)
  #       self.qd_des_log = self.log_policy(self.qd_des_log)
  #       self.qd_log = self.log_policy(self.qd_log)
  #       self.tau_ffwd_log = self.log_policy(self.tau_ffwd_log)
  #       self.tau_log = self.log_policy(self.tau_log)
  #       self.x_ee_log = self.log_policy(self.x_ee_log)
  #       self.x_ee_des_log = self.log_policy(self.x_ee_des_log)
  #       self.contactForceW_log = self.log_policy(self.contactForceW_log)
  #       self.time_log = self.log_policy(self.time_log)
  #   def log_policy(self, var):
  #       tmp = np.empty((var.shape[0], var.shape[1] + conf.robot_params[self.robot_name]['buffer_size'])) * np.nan
  #       tmp[:var.shape[0], :var.shape[1]] = var
  #       return tmp

    def reset_joints(self, q0, joint_names = None):
        # create the message
        req_reset_joints = SetModelConfigurationRequest()
        req_reset_joints.model_name = self.robot_name
        req_reset_joints.urdf_param_name = 'robot_description'
        if joint_names == None:
            req_reset_joints.joint_names = self.joint_names
        else:
            req_reset_joints.joint_names = joint_names
        req_reset_joints.joint_positions = q0
        self.reset_joints_client(req_reset_joints)
        print(colored(f"---------Resetting Joints to: "+str(q0), "blue"))

    def setSimSpeed(self, dt_sim = 0.001, max_update_rate=1000, iters=50):
        physics_req = SetPhysicsPropertiesRequest()
        physics_req.time_step = dt_sim
        physics_req.max_update_rate = max_update_rate
        physics_req.ode_config.sor_pgs_iters = iters
        physics_req.ode_config.sor_pgs_w = 1.3
        physics_req.ode_config.contact_surface_layer = 0.001
        physics_req.ode_config.contact_max_correcting_vel = 100
        physics_req.ode_config.erp = 0.2
        physics_req.ode_config.max_contacts = 20
        physics_req.gravity.z = -9.81
        self.set_physics_client(physics_req)

def talker(p):
    p.start()
    p.startSimulator()
    if ( robotName == 'ur5'):
        p.loadModelAndPublishers(rospkg.RosPack().get_path('ur_description') + '/urdf/' + p.robot_name + '.urdf.xacro')
    else:
        p.loadModelAndPublishers()
    p.initVars()     
    p.startupProcedure()
    ros.sleep(1.0)
    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    #control loop
    while not ros.is_shutdown():
        p.q_des = np.copy(p.q_des_q0)
        # send commands to gazebo
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        # log variables
        p.logData()

        #wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 4) # to avoid issues of dt 0.0009999

if __name__ == '__main__':
    p = BaseControllerFixed(robotName)

    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        if    conf.plotting:
            plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
                       p.tau_ffwd_log, p.joint_names)
            plotJoint('torque', 1, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log,
                    p.tau_ffwd_log, p.joint_names)

        
