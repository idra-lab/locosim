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
from six.moves import input # solves compatibility issue bw pyuthon 2.x and 3 for raw input that does exists in python 3
from termcolor import colored
import time
from base_controllers.utils.common_functions import plotJoint, plotAdmittanceTracking, plotEndeff

#config files
import  base_controllers.params as conf
import L8_conf as lab_conf
robotName = "ur5"

# mother classs
from base_controllers.ur5_generic import Ur5Generic

#lab specific imports
from base_controllers.components.obstacle_avoidance.obstacle_avoidance import ObstacleAvoidance
from base_controllers.components.admittance_controller import AdmittanceControl
from base_controllers.components.trajectory_manager import TrajectoryManager
from geometry_msgs.msg import Pose
from numpy import nan

class LabAdmittanceController(Ur5Generic):
    def __init__(self, robot_name="ur5"):
        super().__init__(robot_name=robot_name)

        if (lab_conf.obstacle_avoidance):
            self.world_name = 'tavolo_obstacles.world'
            if (not self.use_torque_control):
                print(colored("ERRORS: you can use obstacle avoidance only on torque control mode", 'red'))
                sys.exit()
        else:
            #self.world_name = 'tavolo_brick.world'
            #self.world_name = 'palopoli.world'
            self.world_name = None

        if lab_conf.admittance_control and ((not self.real_robot) and (not self.use_torque_control)):
            print(colored("ERRORS: you can use admittance control only on torque control mode or in real robot (need contact force estimation or measurement)", 'red'))
            sys.exit()


        print("Initialized L8 admittance  controller---------------------------------------------------------------")

    def loadModelAndPublishers(self, xacro_path):
        super().loadModelAndPublishers(xacro_path)
        self.traj_manager = TrajectoryManager(self.robot_name, conf.robot_params[self.robot_name], self.controller_manager.gm)
        self.pub_ee_pose = ros.Publisher("/" + self.robot_name + "/ee_pose", Pose, queue_size=10)

    def applyForce(self):
        wrench = Wrench()
        wrench.force.x = 0
        wrench.force.y = 0
        wrench.force.z = 30
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0
        reference_frame = "world" # you can apply forces only in this frame because this service is buggy, it will ignore any other frame
        reference_point = Point(x = 0, y = 0, z = 0)
        try:
            self.apply_body_wrench(body_name="ur5::wrist_3_link", reference_frame=reference_frame, reference_point=reference_point , wrench=wrench, duration=ros.Duration(10))
        except:
            pass

    def estimateContactForces(self):  
        # estimate ground reaction forces from torques tau
        if self.use_torque_control:
            self.contactForceW = np.linalg.inv(self.J6.T).dot(self.h-self.tau)[:3]

    def initVars(self):
        super().initVars()

        # log variables relative to admittance controller
        self.q_des_adm_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.x_ee_des_adm_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.EXTERNAL_FORCE = False
        self.payload_weight_avg = 0.0
        self.polynomial_flag = False
        self.obs_avoidance = ObstacleAvoidance()
        # position of the center of the objects is in WF
        self.obs_avoidance.setCubeParameters(0.25, np.array([0.125, 0.75,0.975]))
        self.obs_avoidance.setCylinderParameters(0.125, 0.3, np.array([0.6, 0.25, 1.0]))
        self.admit = AdmittanceControl(self.ikin, lab_conf.Kx, lab_conf.Dx, conf.robot_params[self.robot_name])
        self.time_poly = None

        if lab_conf.USER_TRAJECTORY:
            self.Q_ref = []
            for name in lab_conf.traj_file_name:
                data = np.load(lab_conf.traj_folder + name + '.npz')
                self.Q_ref.append(data['q'])
            self.ext_traj_counter = 0  # counter for which trajectory is currently tracked
            self.ext_traj_t = 0  # counter for the time inside a trajectory
            self.traj_completed = False


    def logData(self):
        if (conf.robot_params[self.robot_name]['control_type'] == "admittance"):
            self.q_des_adm_log[:, self.log_counter] = self.q_des_adm
            self.x_ee_des_adm_log[:, self.log_counter] = self.x_ee_des_adm
        # I need to do this after because it updates log counter
        super().logData()

    def send_ee_pose(self):
        msg = Pose()
        msg.position = self.x_ee + self.base_offset
        self.pub_ee_pose.publish(msg)

    def plotStuff(self):
        if not (conf.robot_params[p.robot_name]['control_mode'] == "trajectory"):
            if (lab_conf.admittance_control):
                plotJoint('position',  time_log=self.time_log, q_log=self.q_log, q_des_log=self.q_des_log)
                plotAdmittanceTracking(3, self.time_log, self.x_ee_log, self.x_ee_des_log, self.x_ee_des_adm_log, self.contactForceW_log)
            else:
                plotJoint('position', time_log=self.time_log, q_log=self.q_log, q_des_log=self.q_des_log)
            plotJoint('torque', time_log=self.time_log, tau_log=self.tau_log, tau_ffwd_log=self.tau_ffwd_log, joint_names=self.joint_names)
            plotEndeff('force', 1, self.time_log, self.contactForceW_log)


def talker(p):
    p.start()
    if p.real_robot:
        p.startRealRobot()
    else:
        additional_args = ['gripper:=' + str(p.gripper)]  # , 'gui:=false']
        if str(conf.robot_params[p.robot_name]['gripper_type']) == 'soft_2':
            print("setting soft gripper")
            additional_args.append('soft_gripper:=true')
        elif str(conf.robot_params[p.robot_name]['gripper_type']) == 'robotiq_2':
            additional_args.append('robotiq_gripper:=true')
        p.startSimulator(world_name=p.world_name, additional_args=additional_args)

    # specify xacro location
    xacro_path = rospkg.RosPack().get_path('ur_description') + '/urdf/' + p.robot_name + '.urdf.xacro'
    p.loadModelAndPublishers(xacro_path)
    p.initVars()
    p.startupProcedure()

    # sleep to avoid that the real robot crashes on the table
    time.sleep(3.)

    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    p.q_des_q0 = conf.robot_params[p.robot_name]['q_0']
    p.q_des = np.copy(p.q_des_q0)
    p.admit.setPosturalTask(np.copy(p.q_des_q0))


    # # homing procedure
    if p.homing_flag:
        if p.real_robot:
            v_des = 0.2
        else:
            v_des = 3.0
        q_home = conf.robot_params[p.robot_name]['q_0']
        p.homing_procedure(conf.robot_params[p.robot_name]['dt'], v_des,q_home, rate)

    if (conf.robot_params[p.robot_name]['control_mode'] == "trajectory"):
        # to test the trajectory
        p.traj_manager.send_joint_trajectory_2() #send_joint_trajectory_2 manages also gripper
    else:# control_mode =  point
        gripper_on = 0

        #control loop
        while not ros.is_shutdown():
            #update the kinematics
            p.updateKinematicsDynamics()

            if lab_conf.USER_TRAJECTORY: #Del Prete Optimal Trajectory point to point
                if (int(p.ext_traj_t) < p.Q_ref[p.ext_traj_counter].shape[0]): # and p.time>6.0:
                    p.q_des = p.Q_ref[p.ext_traj_counter][int(p.ext_traj_t),:]
                    p.ext_traj_t += 1.0/lab_conf.traj_slow_down_factor
                else:
                    if(p.ext_traj_counter < len(p.Q_ref)-1):
                        print(colored("TRAJECTORY %d COMPLETED"%p.ext_traj_counter, 'blue'))
                        if(p.ext_traj_counter==0):
                            p.controller_manager.gm.move_gripper(30)
                        if (p.ext_traj_counter == 1):
                            p.controller_manager.gm.move_gripper(80)
                            p.controller_manager.gm.move_gripper(30)
                        p.ext_traj_counter += 1
                        p.ext_traj_t = 0
                    elif(not p.traj_completed):
                        print(colored("LAST TRAJECTORY COMPLETED", 'red'))
                        p.controller_manager.gm.move_gripper(80)
                        p.traj_completed = True
                        #ext_traj_t = 0
                        #ext_traj_counter = 0

            # EXE L8-1.1: set constant joint reference
            #p.q_des = np.copy(p.q_des_q0)

            #test gripper
            # if p.gripper:
            #     if p.time > 5.0 and (gripper_on == 0):
            #         print("gripper 30")
            #         p.controller_manager.gm.move_gripper(20)
            #         gripper_on = 1
            #     if (gripper_on == 1) and p.time > 10.0:
            #         print("gripper 100")
            #         p.controller_manager.gm.move_gripper(80)
            #         gripper_on = 2

            # EXE L8-1.2: set sinusoidal joint reference
            #p.q_des  = p.q_des_q0  + lab_conf.amplitude * np.sin(2*np.pi*lab_conf.frequency*p.time)


            # EXE L8-1.3: set constant ee reference
            # p.x_ee_des = lab_conf.ee_reference
            # p.ros_pub.add_marker(p.x_ee_des + p.base_offset, color='blue')
            # p.q_des, ok, out_ws = p.ikin.endeffectorInverseKinematicsLineSearch(p.x_ee_des,
            #                                                                     conf.robot_params[p.robot_name][
            #                                                                         'ee_frame'], p.q, False, False,
            #                                                                     postural_task=True, w_postural=0.00001,
            #                                                                     q_postural=p.q_des_q0)

            # EXE L8-1.5:  set constant ee reference and create polynomial trajectory to reach it
            # p.x_ee_des = lab_conf.ee_reference
            # p.ros_pub.add_marker(p.x_ee_des + p.base_offset, color='blue')
            # if not p.polynomial_flag and p.time > 3.0:
            #     print(colored("STARTING POLYNOMIAL",'red'))
            #     p.time_poly = p.time
            #     p.x_intermediate = np.zeros(3)
            #     a = np.empty((3, 6))
            #     for i in range(3):
            #         a[i, :] = coeffTraj( lab_conf.poly_duration, p.x_ee[i], p.x_ee_des[i])
            #     p.polynomial_flag = True
            # # Polynomial trajectory for x,y,z coordinates
            # if  p.polynomial_flag and (p.time - p.time_poly) <  lab_conf.poly_duration:
            #     t = p.time - p.time_poly
            #     for i in range(3):
            #         p.x_intermediate[i] = a[i, 0] + a[i,1] * t + a[i,2] * pow(t, 2) + a[i,3] * pow(t, 3) + a[i,4]*pow(t, 4) + a[i,5]*pow(t, 5)
            # if p.polynomial_flag:
            #     p.q_des, ok, out_ws = p.ikin.endeffectorInverseKinematicsLineSearch(p.x_intermediate,  conf.robot_params[p.robot_name]['ee_frame'], p.q, False, False,
            #                                                                     postural_task=True, w_postural=0.00001,
            #                                                                     q_postural=p.q_des_q0)

            # EXE L8-1.4:  set constant ee reference and desired orientation
            # rpy_des = lab_conf.des_orient
            # w_R_e_des = p.math_utils.eul2Rot(rpy_des) # compute rotation matrix representing the desired orientation from Euler Angles
            # p.q_des, ok, out_ws = p.ikin.endeffectorFrameInverseKinematicsLineSearch(p.x_ee_des, w_R_e_des, conf.robot_params[p.robot_name]['ee_frame'], p.q)
            # TODO create polynomial for orientation!

            # EXE L8-2 - admittance control
            if (lab_conf.admittance_control):
                p.EXTERNAL_FORCE = True
                ## TODO to implement at the velocity level you need to load the VelocityInterface and use the joint_group_vel_controller
                p.x_ee_des = p.robot.framePlacement(p.q_des,p.robot.model.getFrameId(conf.robot_params[p.robot_name]['ee_frame'])).translation
                p.q_des_adm, p.x_ee_des_adm = p.admit.computeAdmittanceReference(p.contactForceW, p.x_ee_des, p.q)

            # EXE L8-2.5 - load estimation
            # if (p.time > 10.0):
            #     p.payload_weight_avg = 0.99 * p.payload_weight_avg + 0.01 * (-p.contactForceW[2] / 9.81)
            #     print("estimated load: ", p.payload_weight_avg)

            # controller with gravity coriolis comp
            p.tau_ffwd = p.h + np.zeros(p.robot.na)

            # only torque loop (not used)
            #p.tau_ffwd = conf.robot_params[p.robot_name]['kp']*(np.subtract(p.q_des,   p.q))  - conf.robot_params[p.robot_name]['kd']*p.qd

            # override the position command if you use admittance controller
            if (p.time > 1.5) and (lab_conf.admittance_control):  # activate admittance control only after a few seconds to allow initialization
                q_to_send = p.q_des_adm
            else:
                q_to_send = p.q_des

            if (lab_conf.obstacle_avoidance):
                p.tau_ffwd = p.obs_avoidance.computeTorques(p,  lab_conf.des_ee_goal)
            # send commands to gazebo
            p.controller_manager.sendReference(q_to_send, p.qd_des, p.tau_ffwd)

            if(not lab_conf.obstacle_avoidance):
                p.ros_pub.add_arrow(p.x_ee + p.base_offset, p.contactForceW / (6 * p.robot.robot_mass), "green")

            # log variables
            if (p.time > 1.0):
                p.logData()

            # disturbance force
            if (p.time > 10.0 and p.EXTERNAL_FORCE):
                p.applyForce()
                p.EXTERNAL_FORCE = False

            # plot end-effector
            p.ros_pub.add_marker(p.x_ee + p.base_offset)
            p.ros_pub.publishVisual()
            p.send_ee_pose()

            #wait for synconization of the control loop
            rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),  3)  # to avoid issues of dt 0.0009999

if __name__ == '__main__':
    p = LabAdmittanceController(robotName)

    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        if   conf.plotting:
            p.plotStuff()

    
        
