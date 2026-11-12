# -*- coding: utf-8 -*-
"""
Created on 3 May  2022

@author: mfocchi
"""

from __future__ import print_function

import os
import rospy as ros
import sys
# messages for topic subscribers
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger, TriggerRequest

# ros utils
import roslaunch
import rosnode
import rosgraph
import rospkg

#other utils
from base_controllers.utils.math_tools import *
import pinocchio as pin
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
from termcolor import colored
from base_controllers.utils.common_functions import plotJoint, plotEndeff
import  base_controllers.params as conf
robotName = "ur5"

# controller manager management
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from std_msgs.msg import Float64MultiArray
from base_controllers.base_controller_fixed import BaseControllerFixed
import tf
from rospy import Time
import time
from base_controllers.components.controller_manager import ControllerManager
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


class Ur5Generic(BaseControllerFixed):

    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)
        self.real_robot = conf.robot_params[self.robot_name]['real_robot']
        self.homing_flag = True
        if (conf.robot_params[self.robot_name]['control_type'] == "torque"):
            self.use_torque_control = 1
        else:
            self.use_torque_control = 0



        if conf.robot_params[self.robot_name]['gripper_sim']:
            self.gripper = True
        else:
            self.gripper = False

        self.controller_manager = ControllerManager(conf.robot_params[self.robot_name])

        self.world_name = None # only the workbench
        #self.world_name = 'empty.world'
        #self.world_name = 'palopoli.world'

        print("Initialized ur5 generic  controller---------------------------------------------------------------")


    def loadModelAndPublishers(self, xacro_path):
        super().loadModelAndPublishers(xacro_path)

        self.sub_ftsensor = ros.Subscriber("/" + self.robot_name + "/wrench", WrenchStamped,
                                           callback=self._receive_ftsensor, queue_size=1, tcp_nodelay=True)
        self.switch_controller_srv = ros.ServiceProxy(
            "/" + self.robot_name + "/controller_manager/switch_controller", SwitchController)
        self.load_controller_srv = ros.ServiceProxy("/" + self.robot_name + "/controller_manager/load_controller",
                                                    LoadController)
        # specific publisher for joint_group_pos_controller that publishes only position
        self.pub_reduced_des_jstate = ros.Publisher("/" + self.robot_name + "/joint_group_pos_controller/command",
                                                    Float64MultiArray, queue_size=10)

        self.zero_sensor = ros.ServiceProxy("/" + self.robot_name + "/ur_hardware_interface/zero_ftsensor", Trigger)
        self.controller_manager.initPublishers(self.robot_name)
        #  different controllers are available from the real robot and in simulation
        if self.real_robot:
            # specific publisher for joint_group_pos_controller that publishes only position
            self.pub_reduced_des_jstate = ros.Publisher("/" + self.robot_name + "/joint_group_pos_controller/command",
                                                        Float64MultiArray, queue_size=10)
            self.available_controllers = [
                "joint_group_pos_controller",
                "scaled_pos_joint_traj_controller" ]
        else:
            self.available_controllers = ["joint_group_pos_controller",
                                          "pos_joint_traj_controller" ]
        self.active_controller = self.available_controllers[0]

        self.broadcaster = tf.TransformBroadcaster()
        # store in the param server to be used from other planners
        self.utils = Utils()
        self.utils.putIntoGlobalParamServer("gripper_sim", self.gripper)

        self.sub_pointcloud = ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2,   callback=self.receive_pointcloud, queue_size=1)

    def _receive_ftsensor(self, msg):
        contactForceTool0 = np.zeros(3)
        contactMomentTool0 = np.zeros(3)
        contactForceTool0[0] = msg.wrench.force.x
        contactForceTool0[1] = msg.wrench.force.y
        contactForceTool0[2] = msg.wrench.force.z
        contactMomentTool0[0] = msg.wrench.torque.x
        contactMomentTool0[1] = msg.wrench.torque.y
        contactMomentTool0[2] = msg.wrench.torque.z
        self.contactForceW = self.w_R_tool0.dot(contactForceTool0)
        self.contactMomentW = self.w_R_tool0.dot(contactMomentTool0)

    def deregister_node(self):
        print( "deregistering nodes"     )
        self.ros_pub.deregister_node()
        if not self.real_robot:
            os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")
            os.system(" rosnode kill /gzserver /gzclient")
                                                                                                                                     
    def updateKinematicsDynamics(self):
        # q is continuously updated
        # to compute in the base frame  you should put neutral base
        self.robot.computeAllTerms(self.q, self.qd)
        # joint space inertia matrix
        self.M = self.robot.mass(self.q)
        # bias terms
        self.h = self.robot.nle(self.q, self.qd)
        #gravity terms
        self.g = self.robot.gravity(self.q)
        #compute ee position  in the world frame
        frame_name = conf.robot_params[self.robot_name]['ee_frame']
        # this is expressed in the base frame
        self.x_ee = self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).translation
        self.w_R_tool0 = self.robot.framePlacement(self.q, self.robot.model.getFrameId(frame_name)).rotation

        if self.real_robot:
            # zed2_camera_center is the frame where point cloud is generated in REAL robot
            pointcloud_frame = "zed2_camera_center"
        else:
            # left_camera_optical_frame is the frame where point cloud is generated in SIMULATION
            pointcloud_frame = "zed2_left_camera_optical_frame"
        # offset of the camera in the world frame
        self.x_c= self.robot.framePlacement(self.q, self.robot.model.getFrameId(pointcloud_frame)).translation
        self.w_R_c = self.robot.framePlacement(self.q, self.robot.model.getFrameId(pointcloud_frame)).rotation

        # compute jacobian of the end effector in the base or world frame (they are aligned so in terms of velocity they are the same)
        self.J6 = self.robot.frameJacobian(self.q, self.robot.model.getFrameId(frame_name), False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
        # take first 3 rows of J6 cause we have a point contact            
        self.J = self.J6[:3,:] 
        # broadcast base world TF
        self.broadcaster.sendTransform(self.base_offset, (0.0, 0.0, 0.0, 1.0), Time.now(), '/base_link', '/world')


    def startupProcedure(self):
        if (self.use_torque_control):
            #set joint pdi gains
            self.pid.setPDjoints( conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'], np.zeros(self.robot.na))
        if (self.real_robot):
            self.zero_sensor()
        self.u.putIntoGlobalParamServer("real_robot",  self.real_robot)
        print(colored("finished startup -- starting controller", "red"))

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        print('Available controllers: ',self.available_controllers)
        print('Controller manager: loading ', target_controller)

        other_controllers = (self.available_controllers)
        other_controllers.remove(target_controller)
        print('Controller manager:Switching off  :  ',other_controllers)

        srv = LoadControllerRequest()
        srv.name = target_controller

        self.load_controller_srv(srv)  
        
        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers 
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_controller_srv(srv)
        self.active_controller = target_controller

    def deregister_node(self):
        super().deregister_node()
        if not self.real_robot:
            os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")
            os.system(" rosnode kill /gzserver /gzclient")

    def plotStuff(self):
        plotJoint('position', 0, self.time_log, self.q_log, self.q_des_log)

    def homing_procedure(self, dt, v_des, q_home, rate):
        # broadcast base world TF
        self.broadcaster.sendTransform(self.base_offset, (0.0, 0.0, 0.0, 1.0), Time.now(), '/base_link', '/world')
        v_ref = 0.0
        print(colored("STARTING HOMING PROCEDURE", 'red'))
        self.q_des = np.copy(self.q)
        print("Initial joint error = ", np.linalg.norm(self.q_des - q_home))
        print("q = ", self.q.T)
        print("Homing v des", v_des)
        while True:
            e = q_home - self.q_des
            e_norm = np.linalg.norm(e)
            if (e_norm != 0.0):
                v_ref += 0.005 * (v_des - v_ref)
                self.q_des += dt * v_ref * e / e_norm
                self.controller_manager.sendReference(self.q_des)
            rate.sleep()
            if (e_norm < 0.001):
                self.homing_flag = False
                print(colored("HOMING PROCEDURE ACCOMPLISHED", 'red'))
                if self.gripper:
                    p.controller_manager.gm.move_gripper(100)
                break

    def receive_pointcloud(self, msg):
        points_list = []
        #this is the center of the image plane
        center_x = int(msg.width / 2)
        center_y = int(msg.height / 2)

        for data in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=False, uvs=[(center_x, center_y)]):
            points_list.append([data[0], data[1], data[2]])
        #print("Data Optical frame: ", points_list)
        pointW = self.w_R_c.dot(points_list[0]) + self.x_c + self.base_offset
        #print("Data World frame: ", pointW)

def talker(p):
    p.start()
    if p.real_robot:
        p.startRealRobot()
    else:
        additional_args = ['gripper:=' + str(p.gripper), 'soft_gripper:='+ str(conf.robot_params[p.robot_name]['soft_gripper'])]#, 'gui:=false']
        launch_file = rospkg.RosPack().get_path(
            'ros_impedance_controller') + '/launch/ros_impedance_controller_ur5_multiple.launch'

        p.startSimulator(world_name=p.world_name, additional_args =additional_args, launch_file=launch_file)




    # specify xacro location
    xacro_path = rospkg.RosPack().get_path('ur_description') + '/urdf/' + p.robot_name + '.urdf.xacro'
    p.loadModelAndPublishers(xacro_path)
    p.initVars()
    # p.startupProcedure()
    #
    # # sleep to avoid that the real robot crashes on the table
    # time.sleep(3.)

    # loop frequency
    rate = ros.Rate(1 / conf.robot_params[p.robot_name]['dt'])





    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
    p.q_des = np.copy(p.q_des_q0)

    while True:
        rate.sleep()

    # use the point to point position controller
    if not p.use_torque_control:
        p.switch_controller("joint_group_pos_controller")

    # homing procedure
    if p.homing_flag:
        if p.real_robot:
            v_des = 0.2
        else:
            v_des = 0.6
        p.homing_procedure(conf.robot_params[p.robot_name]['dt'], v_des, conf.robot_params[p.robot_name]['q_0'], rate)

    gripper_on = 0

    # launch task planner node (implement the state machine)

    # launch motion node (takes care of moving the end-effector, remember to add a rate.sleep!)

    # launch vision node (visual pipelines to detect object, made with service call)

    #control loop (runs every dt seconds)
    while not ros.is_shutdown():
        p.updateKinematicsDynamics()


        ## set joints here
        # p.q_des = p.q_des_q0  + 0.1 * np.sin(2*np.pi*0.5*p.time)
        # p.qd_des = 0.1 * 2 * np.pi * 0.5* np.cos(2 * np.pi * 0.5 * p.time)*np.ones(p.robot.na)

        ##test gripper
        # in Simulation remember to set gripper_sim : True in params.yaml!
        # if p.time>5.0 and (gripper_on == 0):
        #     print("gripper 30")
        #     p.controller_manager.gm.move_gripper(10)
        #     gripper_on = 1
        # if (gripper_on == 1) and p.time>10.0:
        #     print("gripper 100")
        #     p.controller_manager.gm.move_gripper(100)
        #     gripper_on = 2
        #need to uncomment this to be able to send joints references (leave it commented if you have an external node setting them)
        #p.controller_manager.sendReference(p.q_des, p.qd_des, p.h)

        if p.real_robot:
            p.ros_pub.add_arrow(p.x_ee + p.base_offset, p.contactForceW / (6 * p.robot.robot_mass), "green")

        # log variables
        p.logData()
        # plot end-effector
        p.ros_pub.add_marker(p.x_ee + p.base_offset)
        p.ros_pub.publishVisual()

        #wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),  3)  # to avoid issues of dt 0.0009999

if __name__ == '__main__':

    p = Ur5Generic(robotName)

    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        if   conf.plotting:
            p.plotStuff()

    
        
