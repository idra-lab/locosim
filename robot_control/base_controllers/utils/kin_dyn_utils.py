# -*- coding: utf-8 -*-
"""
Created on May 4 2021

@author: ovillarreal
"""
from __future__ import print_function
import numpy as np
import os
import math
import pinocchio as pin
from pinocchio.utils import *
from base_controllers.utils.math_tools import Math
import time as tm 

def setRobotParameters():

    # link lengths
    l1 = 0.089159 # relative displacement of shoulder link frame along parent (base frame)  Z axis
    l2 = 0.13585  # relative displacement of upper-arm link frame along the parent (shoulder link frame) Y axis
    l3 = 0.425    # relative displacement of forearm link frame along the parent (upper-arm link frame) Z axis
    l4 = 0.1197   # relative displacement of forearm link frame along the parent (upper-arm link frame) Y axis
    l5 = 0.39225  # relative displacement of wrist_1_link frame along the parent (forearm-arm link frame) Z axis
    l6 = 0.094    # relative displacement of ee frame along the parent (wrist_1_link link frame) Y axis
    l7 = 0.068    # relative displacement of ee frame along the parent (wrist_1_link link frame) Z axis
    lengths = np.array([l1, l2, l3, l4, l5, l6, l7])

    m0 = 4  # base link
    m1 = 3.7 # shoulder link
    m2 = 8.393 # upper-arm link
    m3 = 2.275 # forearm link
    m4 = 1.219 # wrist_1_link

    link_masses = np.array([m0, m1, m2, m3, m4])
    
    # com of the links expressed in the respective link frame
    # com_link =  model.inertias[idx].lever (Pinocchio)
    com0 = np.array([0., 0., 0.]) # base link
    com1 = np.array([0., 0., 0.]) # shoulder link
    com2 = np.array([0.  , 0.  , 0.28]) #upper_arm_link
    com3 = np.array([0.  , 0.  , 0.25]) #forearm_link
    com4 = np.array([0., 0., 0.]) # wrist_1_link
    #w_com_link = data.oMi[idx].rotation.dot(com_link) + data.oMi[idx].translation

    # inertia tensors of the links  w.r.t. to own CoM of each link expressed in the respective link frames
    I_0 = np.array([[0.00443333156,           0.0,    0.0],
                    [          0.0, 0.00443333156,    0.0],
                    [          0.0,           0.0, 0.0072]])

    I_1 = np.array([[0.010267495893,            0.0,     0.0],
                    [           0.0, 0.010267495893,     0.0],
                    [           0.0,            0.0, 0.00666]])

    I_2 = np.array([[0.22689067591,            0.0,       0.0],
                    [           0.0, 0.22689067591,       0.0],
                    [           0.0,           0.0, 0.0151074]])
    
    I_3 = np.array([[0.049443313556,            0.0,     0.0],
                    [           0.0, 0.049443313556,     0.0],
                    [           0.0,           0.0, 0.004095]])

    I_4 = np.array([[0.111172755531,            0.0,     0.0],
                    [           0.0, 0.111172755531,     0.0],
                    [           0.0,            0.0, 0.21942]])

    inertia_tensors = np.array([I_0, I_1, I_2, I_3, I_4])

    coms = np.array([com0, com1, com2, com3, com4])

    return lengths, inertia_tensors, link_masses, coms


def directKinematics(q):

    # define link lengths from urdf
    link_length,_,_,_ = setRobotParameters()
    l1 = link_length[0]  # shoulder Z axis
    l2 = link_length[1]  # upper-arm Y axis
    l3 = link_length[2]  # forearm Z axis
    l4 = link_length[3]  # forearm Y axis
    l5 = link_length[4]  # wrist_1_link Z axis
    l6 = link_length[5]  # ee Y axis
    l7 = link_length[6]  # ee Z axis

    q1 = q[0] # shoulder_pan joint position
    q2 = q[1] # shoulder_lift joint position
    q3 = q[2] # elbow joint position
    q4 = q[3] # wrist_1_joint joint position


    # LOCAL homogeneous transformation matrices (base link is 0)

    # shoulder link (1)
    # rigid transform (translation along Z axis)
    T_01r = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, l1],
                       [0, 0, 0, 1]])
    # joint transform (rotation about Z axis)
    T_1r1 = np.array([[math.cos(q1), -math.sin(q1), 0, 0],
                      [math.sin(q1), math.cos(q1),  0, 0],
                      [0,               0,              1, 0],
                      [0,               0,              0, 1]])
    # local hom. transform from link frame 0 to link frame 1
    T_01 = T_01r.dot(T_1r1)



    # upper-arm link (2)
    # rigid transform (90deg rotation about Y axis, translation l2 along Y axis)
    T_12r = np.array([[ 0, 0, 1,  0],
                       [ 0, 1, 0, l2],
                       [-1, 0, 0,  0],
                       [ 0, 0, 0,  1]])
    # joint transform (rotation about Y axis)
    T_2r2 = np.array([[ math.cos(q2), 0, math.sin(q2),        0],
                     [              0, 1,            0,        0],
                     [-math.sin(q2), 0, math.cos(q2), 0],
                     [              0, 0,              0, 1]])
    # local hom. transform from link frame 1 to link frame 2
    T_12 = T_12r.dot(T_2r2)


    # forearm link (3)
    # rigid transform (translation l3 along Zaxis, -l4 along Y axis)
    T_23r = np.array([[1, 0, 0, 0],
                       [0, 1, 0, -l4],
                       [0, 0, 1, l3],
                       [0, 0, 0, 1]])

    # joint transform (rotation about Y axis)
    T_3r3 = np.array([[ math.cos(q3) ,  0, math.sin(q3),   0],
                       [            0,  1,            0,   0],
                       [-math.sin(q3),  0, math.cos(q3),   0],
                       [            0,  0,            0,   1]])
    #local hom. transform from link frame 2 to link frame 3
    T_23 = T_23r.dot(T_3r3)



    # wrist_1 link (4)
    # rigid transform (90 deg about Y axis, l5 translation along Z axis)
    T_34r = np.array([[ 0, 0, 1,  0],
                      [ 0, 1, 0,  0],
                      [ -1, 0, 0,  l5],
                      [ 0, 0, 0,  1]])
    # joint transform  (rotation about Y axis)
    T_4r4 = np.array([[ math.cos(q4), 0, math.sin(q4), 0],
                     [              0, 1,              0, 0],
                     [-math.sin(q4), 0, math.cos(q4), 0],
                     [              0, 0,              0, 1]])
    #local hom. transform from link frame 3 to link frame 4
    T_34 = T_34r.dot(T_4r4)

    # end-effector
    # only rigid transform (rotation X => Y , Y => -X, translation l6 along Y axis, l7 along Z axis)
    T_4e = np.array([[0,  -1, 0,  0],
                     [1,   0, 0, l6],
                     [0,   0, 1, l7],
                     [0,   0, 0,  1]])

    # GLOBAL homogeneous transformation matrices
    T_02 = T_01.dot(T_12)
    T_03 = T_02.dot(T_23)
    T_04 = T_03.dot(T_34)
    T_0e = T_04.dot(T_4e)

    return T_01, T_02, T_03, T_04, T_0e 

'''
    This function computes the Geometric Jacobian of the end-effector expressed in the base link frame 
'''
def computeEndEffectorJacobian(q):

    # compute direct kinematics 
    T_01, T_02, T_03, T_04, T_0e = directKinematics(q)


    # link position vectors
    p_01 = T_01[:3,3]
    p_02 = T_02[:3,3]
    p_03 = T_03[:3,3]
    p_04 = T_04[:3,3]
    p_0e = T_0e[:3,3]

    # z vectors for rotations
    z1 = T_01[:3,2] # Z axis
    z2 = T_02[:3,1] # Y axis
    z3 = T_03[:3,1] # Y axis
    z4 = T_04[:3,1] # Y axis

    # vectors from link i to end-effector
    p_0_1e = p_0e - p_01
    p_0_2e = p_0e - p_02
    p_0_3e = p_0e - p_03
    p_0_4e = p_0e - p_04

    # linear and angular part of Jacobian matrix
    J_p = np.hstack((np.cross(z1,p_0_1e).reshape(3,1) , np.cross(z2,p_0_2e).reshape(3,1) , np.cross(z3,p_0_3e).reshape(3,1) , np.cross(z4,p_0_4e).reshape(3,1)  ))
    J_o = np.hstack((z1.reshape(3,1), z2.reshape(3,1), z3.reshape(3,1), z4.reshape(3,1)))

    # Jacobian matrix and joint axes both expressed in the world frame) 
    J = np.vstack(( J_p, J_o))

    return J,z1,z2,z3,z4


def geometric2analyticJacobian(J,T_0e):
    R_0e = T_0e[:3,:3]
    math_utils = Math()
    rpy_ee = math_utils.rot2eul(R_0e)
    roll = rpy_ee[0]
    pitch = rpy_ee[1]
    yaw = rpy_ee[2]

    # compute the mapping between euler rates and angular velocity
    T_w = np.array([[math.cos(yaw)*math.cos(pitch),  -math.sin(yaw), 0],
                    [math.sin(yaw)*math.cos(pitch),   math.cos(yaw), 0],
                    [             -math.sin(pitch),               0, 1]])

    T_a = np.array([np.vstack((np.hstack((np.identity(3), np.zeros((3,3)))),
                                          np.hstack((np.zeros((3,3)),np.linalg.inv(T_w)))))])


    J_a = np.dot(T_a, J)

    return J_a[0]

def numericalInverseKinematics(p_d, q0, line_search = False, wrap = False):
    math_utils = Math()

    # hyper-parameters
    epsilon = 1e-06 # Tolerance for stopping criterion
    lambda_ = 1e-08  # Regularization or damping factor (1e-08->0.01)
    max_iter = 200  # Maximum number of iterations
    # For line search only
    #gamma = 0.5
    beta = 0.5 # Step size reduction

    # initialization of variables
    iter = 0
    alpha = 1  # Step size
    log_grad = []
    log_err = []

    # Inverse kinematics with line search
    while True:
        # evaluate  the kinematics for q0
        J,_,_,_,_ = computeEndEffectorJacobian(q0)
        _, _, _, _, T_0e = directKinematics(q0)

        p_e = T_0e[:3,3]
        R = T_0e[:3,:3]
        rpy = math_utils.rot2eul(R)
        roll = rpy[0]
        p_e = np.append(p_e,roll)

        # error
        e_bar = p_e - p_d
        J_bar = geometric2analyticJacobian(J,T_0e)
        # take first 4 rows correspondent to our task
        J_bar = J_bar[:4,:]
        # evaluate the gradient
        grad = J_bar.T.dot(e_bar)

        log_grad.append(np.linalg.norm(grad))
        log_err.append(np.linalg.norm(e_bar))

        if np.linalg.norm(grad) < epsilon:
            print("IK Convergence achieved!, norm(grad) :", np.linalg.norm(grad) )
            print("Inverse kinematics solved in {} iterations".format(iter))     
            break
        if iter >= max_iter:                
            print("Warning: Max number of iterations reached, the iterative algorithm has not reached convergence to the desired precision. Error is:  ", np.linalg.norm(e_bar))
            break
        # Compute the error
        JtJ= np.dot(J_bar.T,J_bar) + np.identity(J_bar.shape[1])*lambda_
        JtJ_inv = np.linalg.inv(JtJ)
        P = JtJ_inv.dot(J_bar.T)
        dq = - P.dot(e_bar)

        if not line_search:
            q1 = q0 + dq * alpha
            q0 = q1
        else:
            print("Iter # :", iter)
            # line search loop
            while True:
                #update
                q1 = q0 + dq*alpha
                # evaluate  the kinematics for q1
                _, _, _, _, T_0e1 = directKinematics(q1)
                p_e1 = T_0e1[:3,3]
                R1 = T_0e1[:3,:3]
                rpy1 = math_utils.rot2eul(R1)
                roll1 = rpy1[0]
                p_e1 = np.append(p_e1,roll1)
                e_bar_new = p_e1 - p_d
                #print "e_bar1", np.linalg.norm(e_bar_new), "e_bar", np.linalg.norm(e_bar)

                error_reduction = np.linalg.norm(e_bar) - np.linalg.norm(e_bar_new)
                threshold = 0.0 # more restrictive gamma*alpha*np.linalg.norm(e_bar)

                if error_reduction < threshold:
                    alpha = beta*alpha
                    print (" line search: alpha: ", alpha)
                else:
                    q0 = q1
                    alpha = 1
                    break

        iter += 1
           

 
    # wrapping prevents from outputs outside the range -2pi, 2pi
    if wrap:
        for i in range(len(q0)):
            while q0[i] >= 2 * math.pi:
                q0[i] -= 2 * math.pi
            while q0[i] < -2 * math.pi:
                q0[i] += 2 * math.pi

    return q0, log_err, log_grad


def fifthOrderPolynomialTrajectory(tf,start_pos,end_pos, start_vel = 0, end_vel = 0, start_acc =0, end_acc = 0):

    # Matrix used to solve the linear system of equations for the polynomial trajectory
    polyMatrix = np.array([[1,  0,              0,               0,                  0,                0],
                           [0,  1,              0,               0,                  0,                0],
                           [0,  0,              2,               0,                  0,                0],
                           [1, tf,np.power(tf, 2), np.power(tf, 3),    np.power(tf, 4),  np.power(tf, 5)],
                           [0,  1,           2*tf,3*np.power(tf,2),   4*np.power(tf,3), 5*np.power(tf,4)],
                           [0,  0,              2,             6*tf, 12*np.power(tf,2),20*np.power(tf,3)]])
    
    polyVector = np.array([start_pos, start_vel, start_acc, end_pos, end_vel, end_acc])
    matrix_inv = np.linalg.inv(polyMatrix)
    polyCoeff = matrix_inv.dot(polyVector)

    return polyCoeff
    
def RNEA(g0,q,qd,qdd, Fee = np.zeros(3), Mee = np.zeros(3), joint_types = ['revolute', 'revolute','revolute','revolute']):

    # setting values of inertia tensors w.r.t. to their CoMs from urdf and link masses
    _, tensors, m, coms = setRobotParameters()

    # get inertia tensors about the CoM expressed in the respective link frame
    _0_I_0 = tensors[0]
    _1_I_1 = tensors[1]
    _2_I_2 = tensors[2]
    _3_I_3 = tensors[3]
    _4_I_4 = tensors[4]
    
    # get positions of the link CoM expressed in the respective link frame
    _0_com_0 = coms[0]
    _1_com_1 = coms[1]
    _2_com_2 = coms[2]
    _3_com_3 = coms[3]
    _4_com_4 = coms[4]


    # number of joints
    n = len(q)
    
    #pre-pend a fake joint for base link
    q_link = np.insert(q, 0, 0.0, axis=0)
    qd_link = np.insert(qd, 0, 0.0, axis=0)
    qdd_link = np.insert(qdd, 0, 0.0, axis=0)
        
    # initialation of variables
    zeroV = np.zeros(3)
    omega = np.array([zeroV, zeroV, zeroV, zeroV, zeroV])
    v = np.array([zeroV, zeroV, zeroV, zeroV, zeroV])
    omega_dot = np.array([zeroV, zeroV, zeroV, zeroV,zeroV])
    a = np.array([zeroV, zeroV, zeroV, zeroV, zeroV])
    vc = np.array([zeroV, zeroV, zeroV, zeroV, zeroV])
    ac = np.array([zeroV, zeroV, zeroV, zeroV,zeroV])

    # these arrays are 1 element longer than the others because in the back recursion we consider also the forces/moments coming from the ee
    F = np.array([zeroV, zeroV, zeroV, zeroV, zeroV, Fee])
    M = np.array([zeroV, zeroV, zeroV, zeroV, zeroV, Mee])

    effort = np.array([0.0, 0.0, 0.0, 0.0])

    # obtaining joint axes vectors required in the computation of the velocities and accelerations (expressed in the world frame)
    _,z1,z2,z3,z4 = computeEndEffectorJacobian(q)

    z = np.array([np.zeros(3), z1,z2,z3,z4])

    # global homogeneous transformation matrices
    T_01, T_02, T_03, T_04, T_0e = directKinematics(q)

    # link positions w.r.t. the world frame
    p_00 = np.array([0.0,0.0,0.0])
    p_01 = T_01[:3,3]
    p_02 = T_02[:3,3]
    p_03 = T_03[:3,3]
    p_04 = T_04[:3,3]
    p_0e = T_0e[:3,3]

    # array used in the recursion (this array is 1 element longer than the others because in the back recursion we consider also the position of the ee)
    p = np.array([p_00, p_01, p_02, p_03, p_04, p_0e])

    # rotation matrices w.r.t. to the world of each link
    R_00 = np.eye(3)    
    R_01 = T_01[:3,:3]
    R_02 = T_02[:3,:3]
    R_03 = T_03[:3,:3]
    R_04 = T_04[:3,:3]

    # positions of the CoMs w.r.t. to the world frame
    pc_0 = p_00 + _0_com_0
    pc_1 = p_01 + np.dot(R_01, _1_com_1)
    pc_2 = p_02 + np.dot(R_02, _2_com_2)
    pc_3 = p_03 + np.dot(R_03, _3_com_3)
    pc_4 = p_04 + np.dot(R_04, _4_com_4)

    # array used in the recursion
    pc = np.array([pc_0, pc_1, pc_2, pc_3, pc_4])

    # expressing tensors of inertia of the links (about the com) in the world frame (time consuming)
    I_0 = np.dot(np.dot(R_00,_0_I_0),R_00.T)
    I_1 = np.dot(np.dot(R_01,_1_I_1),R_01.T)
    I_2 = np.dot(np.dot(R_02,_2_I_2),R_02.T)
    I_3 = np.dot(np.dot(R_03,_3_I_3),R_03.T)
    I_4 = np.dot(np.dot(R_04,_4_I_4),R_04.T)

    # array used in the recursion
    I = np.array([I_0, I_1, I_2, I_3, I_4])

    # forward pass: compute accelerations from link 0 to  link 4, range(n+1) = (0, 1, 2, 3, 4)
    for i in range(n+1):
        joint_idx = i-1
        if i == 0: # we start from base link 0
            p_ = p[0]
            #base frame is still (not true for a legged robot!)
            omega[0] = zeroV
            v[0] = zeroV
            omega_dot[0] = zeroV
            a[0] = -g0 # if we consider gravity as  acceleration (need to move to right hand side of the Newton equation) we can remove it from all the Netwon equations
        else:
            if joint_types[joint_idx] == 'prismatic':  # prismatic joint
                p_ = p[i] - p[i - 1]
                omega[i] = omega[i - 1]
                omega_dot[i] = omega_dot[i - 1]
                v[i] = v[i - 1] + qd_link[i] * z[i] + np.cross(omega[i], p_)
                a[i] = a[i - 1] + qdd_link[i] * z[i] + 2 * qd_link[i] * np.cross(omega[i], z[i]) + np.cross(omega_dot[i], p_) + np.cross(omega[i], np.cross(omega[i], p_))
            elif joint_types[joint_idx] == 'revolute':
                p_ = p[i] - p[i - 1]
                omega[i] = omega[i - 1] + qd_link[i] * z[i]
                omega_dot[i] = omega_dot[i - 1] + qdd_link[i] * z[i] + qd_link[i] * np.cross(omega[i - 1], z[i])
                v[i] = v[i - 1] + np.cross(omega[i - 1], p_)
                a[i] = a[i - 1] + np.cross(omega_dot[i - 1], p_) + np.cross(omega[i - 1], np.cross(omega[i - 1], p_))
            else:
                print("wrong joint type")
        pc_ = pc[i] - p[i] # p_i,c
        
        #compute com quantities
        vc[i] = v[i] + np.cross(omega[i],p_)
        ac[i] = a[i] + np.cross(omega_dot[i],pc_) + np.cross(omega[i],np.cross(omega[i],pc_))

    
    # backward pass: compute forces and moments from wrist link (4) to base link (0)
    for i in range(n,-1,-1):   
        # lever arms wrt to other link frames
        pc_ = p[i] - pc[i]
        pc_1 = p[i+1] - pc[i] 
        
        F[i] = F[i+1] + m[i]*(ac[i])
        
        M[i] = M[i+1] - \
               np.cross(pc_,F[i]) + \
               np.cross(pc_1,F[i+1]) + \
               np.dot(I[i],omega_dot[i]) + \
               np.cross(omega[i],np.dot(I[i],omega[i]))  

    # compute torque for all joints (revolute) by projection
    for joint_idx in range(n):
        if joint_types[joint_idx] == 'prismatic':
            effort[joint_idx] = np.dot(z[joint_idx + 1], F[joint_idx + 1])
        elif joint_types[joint_idx] == 'revolute':
            effort[joint_idx] = np.dot(z[joint_idx + 1], M[joint_idx + 1])
        else:
            print("wrong joint type")
    return effort

# computation of gravity terms
def getg(q,robot, joint_types = ['revolute', 'revolute','revolute','revolute']):
    qd = np.array([0.0, 0.0, 0.0, 0.0])
    qdd = np.array([0.0, 0.0, 0.0, 0.0])
    # Pinocchio
    # g = pin.rnea(robot.model, robot.data, q,qd ,qdd)
    g = RNEA(np.array([0.0, 0.0, -9.81]),q,qd,qdd, joint_types=joint_types)
    return g


# computation of generalized mass matrix
def getM(q,robot, joint_types = ['revolute', 'revolute','revolute','revolute']):
    n = len(q)
    M = np.zeros((n,n))
    for i in range(n):
        ei = np.array([0.0, 0.0, 0.0, 0.0])
        ei[i] = 1
        # Pinocchio
        #g = getg(q,robot)
        # tau_p = pin.rnea(robot.model, robot.data, q, np.array([0,0,0,0]),ei) -g      
        tau = RNEA(np.array([0.0, 0.0, 0.0]), q, np.array([0.0, 0.0, 0.0, 0.0]),ei, joint_types=joint_types)
        # fill in the column of the inertia matrix
        M[:4,i] = tau        
        
    return M

def getC(q,qd,robot, joint_types = ['revolute', 'revolute','revolute','revolute']):
    qdd = np.array([0.0, 0.0, 0.0, 0.0])
    # Pinocchio
    # g = getg(q,robot)
    # C = pin.rnea(robot.model, robot.data,q,qd,qdd) - g    
    C = RNEA(np.array([0.0, 0.0, 0.0]), q, qd, qdd, joint_types=joint_types)
    return C      

    



