# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

#!/usr/bin/env python

#inherit from base controller
import time

from base_controllers.base_controller import BaseController
import rospy as ros 
import numpy as np
from numpy import nan
import copy

# L5 Controller specific
from base_controllers.components.controlRoutines import projectionBasedController, QPController
from base_controllers.utils.common_functions import plotFrame, plotContacts, plotConstraitViolation, plotJoint
from scipy.linalg import block_diag
from base_controllers.utils.math_tools import motionVectorTransform
from base_controllers.utils.common_functions import State
import matplotlib.pyplot as plt

import base_controllers.params as conf
import L6_conf as lab_conf
robotName = "hyq"

class Params:
    pass 
des_state = State(desired = True)
act_state = State() 

class AdvancedController(BaseController): 

    def __init__(self, robot_name="hyq"):
        super().__init__(robot_name=robot_name)

    def initVars(self):
        super().initVars()

        self.des_PoseW_log = np.empty((6, conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.des_Twist_log = np.empty((6,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.des_Acc_log = np.empty((6,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.des_forcesW_log = np.empty((12,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.comPoseW_log = np.empty((6, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Wffwd_log = np.empty((6, conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.Wfbk_log = np.empty((6,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.Wg_log = np.empty((6,conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.constr_viol_log = np.empty((4,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.two_pi_f             = 2*np.pi*lab_conf.freq   # 2 PI * frequency
        self.two_pi_f_amp         = np.multiply(p.two_pi_f, lab_conf.amp) # A * 2 PI * frequency
        self.two_pi_f_squared_amp = np.multiply(p.two_pi_f, p.two_pi_f_amp)  # A * (2 PI * frequency)^2

        self.params = Params()

    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            self.des_PoseW_log[:, self.log_counter] = self.des_poseW
            self.des_Twist_log[:, self.log_counter] =  self.des_twist
            self.des_Acc_log[:, self.log_counter] =  self.des_acc
            self.comPoseW_log[:, self.log_counter] = self.comPoseW
            self.des_forcesW_log[:, self.log_counter] =  self.des_forcesW
            self.Wffwd_log[:, self.log_counter] =  self.Wffwd            
            self.Wfbk_log[:, self.log_counter] =  self.Wfbk            
            self.Wg_log[:, self.log_counter] =  self.Wg
            if hasattr(self,'constr_viol'):
                self.constr_viol_log[:, self.log_counter] =  self.constr_viol
        super().logData()

def talker(p):
    p.start()
    #p.startSimulator(additional_args=['gui:=false'])
    p.startSimulator(world_name='slow.world', additional_args=['gui:=false'])
    p.loadModelAndPublishers()
    p.initVars()
    p.initSubscribers()
    p.startupProcedure()
    p.updateKinematics()

    rate = ros.Rate(1/lab_conf.dt) # 10hz

    p.params.isCoMControlled = False
    #EXERSISE 5:
    #p.params.isCoMControlled = True

    # Reset reference to actual value
    if p.params.isCoMControlled:
        p.x0 = np.copy(p.comPoseW)
    else:
        p.x0 = np.copy(p.basePoseW)

    # ensure PDs are zero to avoid conflict with constant joint trajectories
    p.pid.setPDs(0.0, 0.0, 0.0)

    # Control loop               
    while  (p.time  < lab_conf.exp_duration) or (lab_conf.CONTINUOUS and not ros.is_shutdown()):
        start = time.time()
        #update the kinematics
        p.updateKinematics()

        # EXERCISE 1: Sinusoidal Reference Generation
        # Reference Generation
        # for user convenience we assume the XYZ
        des_euler_angles = (p.x0 +  lab_conf.amp*(np.cos(p.two_pi_f*p.time + lab_conf.phi) -1))[3:6]
        w_R_des_hf = p.math_utils.eul2Rot(np.array([0,0,des_euler_angles[2]]))
        p.des_poseW = np.zeros(6)
        p.des_poseW[0:3] = w_R_des_hf @ ((p.x0 +  lab_conf.amp*(np.cos(p.two_pi_f*p.time + lab_conf.phi) -1))[0:3])
        p.des_poseW[3:6] = des_euler_angles
        # IMPORTANT! these is improperly called des_twist des_acc but we are filling the angular part with euler_rates and derivative of euler rates!
        p.des_twist = np.zeros(6)
        p.des_twist[0:3]  = w_R_des_hf @ (( -p.two_pi_f_amp * np.sin(p.two_pi_f*p.time + lab_conf.phi))[0:3])
        p.des_twist[3:6] = ( -p.two_pi_f_amp * np.sin(p.two_pi_f*p.time + lab_conf.phi))[3:6]
        p.des_acc = np.zeros(6)
        p.des_acc[0:3]  = w_R_des_hf @ ((-p.two_pi_f_squared_amp * np.cos(p.two_pi_f*p.time + lab_conf.phi))[0:3])
        p.des_acc[3:6] = (-p.two_pi_f_squared_amp * np.cos(p.two_pi_f*p.time + lab_conf.phi))[3:6]

        #use this to compute acceleration for a custom trajectory
        #des_acc = np.subtract(des_twist, p.des_twist_old)/p.Ts
        #p.des_twist_old = des_twist

        # EXERCISE 6: Check static stability, move CoM out of the polygon   
        #p.des_pose[p.u.sp_crd["LY"]] +=0.0004
    
        # EXERCISE 8.a: Swift the Com on triangle of LF, RF, LH
#        p.des_pose[p.u.sp_crd["LX"]] = 0.1
#        p.des_pose[p.u.sp_crd["LY"]] = 0.1 
        # EXERCISE 8.b: Unload RH leg 
#        if p.time > 2.0:                            
#            p.stance_legs[p.u.leg_map["RH"]] = False       
   
        #time1 = time.time()-startTime #3 ms
        des_state.pose.set(p.des_poseW)
        des_state.twist.set(p.des_twist)
        des_state.accel.set(p.des_acc)

        # offset of the com wrt base origin in WF
        p.params.W_base_to_com = p.u.linPart(p.comPoseW) - p.u.linPart(p.basePoseW)
        p.params.robot = p.robot

        if not p.params.isCoMControlled:
            act_state.pose.set(p.basePoseW)
            act_state.twist.set(p.baseTwistW)
            p.params.robotInertiaB = p.centroidalInertiaB
        else:
            act_state.pose.set(p.comPoseW)
            act_state.twist.set(p.comTwistW)
            p.params.robotInertiaB = p.compositeRobotInertiaB


        #################################################################
        # compute desired contact forces from the whole-body controller                      
        #################################################################
          

        # EXERCISE 2: Projection-based controller (base frame)
#        p.params.gravityComp = False
#        p.params.ffwdOn = False
#        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = projectionBasedController(lab_conf.control_params[p.robot_name], act_state, des_state, p.W_contacts, p.stance_legs, p.params)

        # EXERCISE 3: Add Gravity Compensation (base frame)        
#        p.params.gravityComp = True
#        p.params.ffwdOn = False
#        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = projectionBasedController(lab_conf.control_params[p.robot_name], act_state, des_state, p.W_contacts, p.stance_legs, p.params)
#     
        # EXERCISE 4: Add FFwd Term (base frame) 
#        p.params.gravityComp = True
#        p.params.ffwdOn = True
#        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = projectionBasedController(lab_conf.control_params[p.robot_name], act_state, des_state, p.W_contacts, p.stance_legs, p.params)
#                                
#        # EXERSISE 5: Projection-based controller (CoM)    
        p.params.gravityComp = True
        p.params.ffwdOn = True
        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg = projectionBasedController(lab_conf.control_params[p.robot_name], act_state, des_state, p.W_contacts, p.stance_legs, p.params)


        # EXERCISE 7: quasi-static QP controller (base frame) - unilateral constraints                
        # p.params.normals = [None]*4
        # p.params.normals[p.u.leg_map["LF"]] = np.array([0.0,0.0,1.0])
        # p.params.normals[p.u.leg_map["LH"]] = np.array([0.0,0.0,1.0])
        # p.params.normals[p.u.leg_map["RF"]] = np.array([0.0,0.0,1.0])
        # p.params.normals[p.u.leg_map["RH"]] = np.array([0.0,0.0,1.0])
        # p.params.f_min = np.array([0.0,0.0,0.0, 0.0])
        # p.params.friction_coeff = np.array([0.6,0.6,0.6, 0.6])
        #
        # p.params.gravityComp = True
        # p.params.ffwdOn = True
        # p.params.frictionCones = False
        #
        # p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg, p.constr_viol =  QPController(lab_conf.control_params[p.robot_name], act_state, des_state, p.W_contacts, p.stance_legs, p.params)

        # EXERCISE 9: quasi-static QP controller (base frame) - friction cone constraints                                    
        #p.params.frictionCones = True
        #p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg, p.constr_viol =  QPController(lab_conf.control_params[p.robot_name], act_state, des_state, p.W_contacts, p.stance_legs, p.params)

        # time2 = time.time()-startTime #4 ms

        #################################################################
        # map desired contact forces into torques (missing gravity compensation)                      
        #################################################################                                       
        p.jacsT = block_diag(np.transpose(p.wJ[p.u.leg_map["LF"]]),
                             np.transpose(p.wJ[p.u.leg_map["LH"]] ),
                             np.transpose(p.wJ[p.u.leg_map["RF"]] ),
                             np.transpose(p.wJ[p.u.leg_map["RH"]]  ))
        p.tau_ffwd =    p.h_joints - p.jacsT.dot(p.des_forcesW)
 
    
        #time3 = time.time()-startTime #0.3 ms
       # send desired command to the ros controller     
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

        p.logData()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]),   3)  # to avoid issues of dt 0.0009999

        # plot actual (green) and desired (blue) contact forces 
        for leg in range(4):
            p.ros_pub.add_arrow(p.W_contacts[leg], p.u.getLegJointState(leg, p.grForcesW/(5*p.robot.robotMass)),"green")
            p.ros_pub.add_arrow(p.W_contacts[leg], p.u.getLegJointState(leg, p.des_forcesW/(5*p.robot.robotMass)),"blue")
        p.ros_pub.publishVisual()

        #time4 = time.time() - startTime # 3ms
        #wait for synconization of the control loop
        rate.sleep()

if __name__ == '__main__':
    p = AdvancedController(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
    finally:
        ros.signal_shutdown("killed")
        p.deregister_node()
        if conf.plotting:
            if not p.params.isCoMControlled:
                plotFrame('position', time_log=p.time_log, des_Pose_log=p.des_PoseW_log, Pose_log=p.basePoseW_log,
                          title='base lin/ang position', frame='W', sharex=True, sharey=False, start=0, end=-1)

            else:
                plotFrame('position', time_log=p.time_log, des_Pose_log=p.des_PoseW_log, Pose_log=p.comPoseW_log,
                      title='com lin/ang position', frame='W', sharex=True, sharey=False, start=0, end=-1)

            #plot des_acc (desired derivetive of euler rates) and des_twist(desired euler rates)
            plotFrame('acceleration', time_log=p.time_log, Acc_log=p.des_Acc_log,
                      title='acceleration', frame='W', sharex=True, sharey=False, start=0, end=-1)
            plotFrame('velocity', time_log=p.time_log, Twist_log=p.des_Twist_log,
                      title='velocity', frame='W', sharex=True, sharey=False, start=0, end=-1)
            plotFrame('wrench', time_log=p.time_log, Wrench_log=p.Wffwd_log,
                      title='wrench', frame='W', sharex=True, sharey=False, start=0, end=-1)

            plotContacts('GRFs', time_log=p.time_log, des_Forces_log=p.des_forcesW_log, Forces_log=p.grForcesW_log, frame='W',
                         sharex=True, sharey=False, start=0, end=-1)
            # plotConstraitViolation(3,p.constr_viol_log)
            # plotJoint('torque',4, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log, p.tau_ffwd_log)
