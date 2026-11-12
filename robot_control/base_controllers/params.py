# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np

robot_params = {}

robot_params['myrobot'] ={'dt': 0.001,
                        'kp': np.array([10.,   10.,    10.,  10.]),
                        'kd':  np.array([1.,    1.,    1.,   1.  ]),
                        'q_0':  np.array([0, 0, 0, 0]),
                        'joint_names': ['lf_shoulder_pan', 'lh_shoulder_pan',  'rf_shoulder_pan', 'rh_shoulder_pan'],
                        'ee_frames': ['lf_foot', 'lh_foot', 'rf_foot','rh_foot'],
                        'spawn_x': 0.0,
                        'spawn_y': 0.0,
                        'spawn_z': 1.0,
                        'buffer_size': 30001}

robot_params['ur5'] ={'dt': 0.001,
                       'kp': np.array([300, 300, 300,30,30,1]),
                       'kd':  np.array([20,20,20,5, 5,0.5]),
                       'ki':  np.array([.0,.0,.0,.0, .0,.0]),
                       'q_0':  np.array([ -0.32,-0.78, -2.56,-1.63, -1.57, 3.49]),
                       'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
                       'ee_frame': 'tool0',
                       'control_mode': 'point', # 'trajectory','point'
                       'real_robot': False,
                       'control_type': 'position', # 'position', 'torque'
                       'gripper_sim': False, # False: the gripper is treated as a Rigid Body
                       'gripper_type': 'soft_2', # 3 finger rigid gripper: 'hard',  2 finger soft gripper: 'soft_2',
                                                    # robotiq gripper: 'robotiq_2' (does not work in torque mode!)
                       'spawn_x' : 0.5,
                       'spawn_y' : 0.35,
                       'spawn_z' : 1.75,
                       'buffer_size': 5000}



robot_params['solo'] ={'dt': 0.002,
                       'kp': [5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 5., 5.],
                       'kd': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                       'q_0':  np.array([0,  np.pi/4, -np.pi/2,    0, -np.pi/4,  np.pi/2, -0,  np.pi/4, -np.pi/2, -0, -np.pi/4,  np.pi/2]),
                       'q_fold': np.array([0,  1.57, -3.13, 0,    1.57, -3.13, 0, -1.57, 3.13, 0, -1.57, 3.13]),
                       'joint_names': ['lf_haa_joint', 'lf_hfe_joint', 'lf_kfe_joint',
                                       'lh_haa_joint', 'lh_hfe_joint', 'lh_kfe_joint',
                                       'rf_haa_joint', 'rf_hfe_joint', 'rf_kfe_joint',
                                       'rh_haa_joint', 'rh_hfe_joint', 'rh_kfe_joint'],
                       'ee_frames': ['lf_foot', 'lh_foot', 'rf_foot','rh_foot'],
                       'real_robot': False,
                       'force_th': 2,
                       'spawn_x': 0.0,
                       'spawn_y': 0.0,
                       'spawn_z': 0.3,
                        'buffer_size': 1501} # note the frames are all aligned with base for joints = 0

verbose = False
plotting = True

