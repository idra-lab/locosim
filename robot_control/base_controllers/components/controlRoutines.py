# Description
# File contains some necessary control algorithms for HyQ
# Author: Niraj Rathod
# Date: 19-11-2019

# Standard packages
import scipy.io
import scipy.sparse as sparse
import numpy as np
import yaml
from base_controllers.utils.math_tools import *
# User defined packages
from base_controllers.utils.utils import Utils

from base_controllers.utils.optimTools import quadprog_solve_qp
from scipy.linalg import block_diag

def computeVirtualImpedanceWrench(conf, act_state, des_state, W_contacts, stance_legs, params):
    util = Utils()

    # Load math functions 
    mathJet = Math()

    # actual orientation
    w_R_b = mathJet.eul2Rot(act_state.pose.orientation)
    # Desired Orientation
    w_R_des = mathJet.eul2Rot(des_state.pose.orientation)

    w_R_hf = mathJet.eul2Rot((np.array([0, 0, act_state.pose.orientation[2]])))
    # Feedback wrench (Virtual PD)
    # assume gains are given in BF for convenience
    Kp_lin = w_R_hf.T @ np.diag([conf['Kp_lin_x'], conf['Kp_lin_y'], conf['Kp_lin_z']]) @ w_R_hf
    Kd_lin = w_R_hf.T @ np.diag([conf['Kd_lin_x'], conf['Kd_lin_x'], conf['Kd_lin_x']]) @ w_R_hf
    Kp_ang = w_R_hf.T @ np.diag([conf['KpRoll'], conf['KpPitch'], conf['KpYaw']]) @ w_R_hf
    Kd_ang = w_R_hf.T @  np.diag([conf['KdRoll'], conf['KdPitch'], conf['KdYaw']]) @ w_R_hf

    Wfbk = np.zeros(6)
                
    # linear part                
    Wfbk[util.sp_crd["LX"]:util.sp_crd["LX"] + 3] = Kp_lin.dot(des_state.pose.position - act_state.pose.position) + Kd_lin.dot(des_state.twist.linear - act_state.twist.linear)
    # angular part                
    w_err = computeOrientationError(w_R_b, w_R_des)

    # map des euler tates into des omega
    Jomega =  mathJet.Tomega(act_state.pose.orientation)
   
    # Note we defined the angular part of the des_twist as euler rates not as omega so we need to map them to an Euclidean space with Jomega
    Wfbk[util.sp_crd["AX"]:util.sp_crd["AX"] + 3] = Kp_ang.dot(w_err) + Kd_ang.dot(Jomega.dot(des_state.twist.angular) - act_state.twist.angular)


    #EXERCISE 3: Compute graviy wrench
    Wg = np.zeros(6)
    if (params.gravityComp == True):
        mg = params.robot.robotMass * np.array([0, 0, conf['gravity']])
        Wg[util.sp_crd["LX"]:util.sp_crd["LX"] + 3] = mg
        #in case you are closing the loop on base frame
        if (not params.isCoMControlled):  # act_state  = base position in this case
            Wg[util.sp_crd["AX"]:util.sp_crd["AX"] + 3] = np.cross(params.W_base_to_com, mg)
                    
    # EXERCISE 4: Feed-forward wrench
    Wffwd = np.zeros(6)                                                                
    if (params.ffwdOn):    
        ffdLinear = params.robot.robotMass * des_state.accel.linear
       # compute inertia in the WF  w_I = R' * B_I * R
        W_Inertia = np.dot(w_R_b, np.dot(params.robotInertiaB, w_R_b.T))
        # compute w_des_omega_dot  Jomega*des_euler_rates_dot + Jomega_dot*des euler_rates
        Jomega_dot =  mathJet.Tomega_dot(des_state.pose.orientation,  des_state.twist.angular)
        w_des_omega_dot = Jomega.dot(des_state.accel.angular) + Jomega_dot.dot(des_state.twist.angular)                
        ffdAngular = W_Inertia.dot(w_des_omega_dot) 
        #ffdAngular = W_Inertia.dot(util.angPart(des_state.des_acc))
        Wffwd = np.hstack([ffdLinear, ffdAngular])
        

    return  Wffwd, Wfbk, Wg           
                 
# Whole body controller for HyQ that includes ffd wrench + fb Wrench (Virtual PD) + gravity compensation
# all vector is in the wf
def projectionBasedController(conf, act_state, des_state, W_contacts, stance_legs,  params):
    util = Utils()
                       
    # EXERCISE 2.1: compute virtual impedances  
    Wffwd, Wfbk, Wg = computeVirtualImpedanceWrench(conf, act_state, des_state,  W_contacts,  stance_legs,  params)                
    # Total Wrench (FFwd + Feedback + Gravity)
    TotWrench =  Wffwd + Wfbk + Wg                     
                
    # EXERCISE 2.2: Compute mapping to grfs
    # Stance matrix
    S_mat = np.diag(np.hstack([stance_legs[util.leg_map["LF"]] * np.ones(3),
                               stance_legs[util.leg_map["LH"]] * np.ones(3),
                               stance_legs[util.leg_map["RF"]] * np.ones(3),
                               stance_legs[util.leg_map["RH"]] * np.ones(3)]))
    # This is a skew symmetric matrix for (xfi-xc)  corressponding  toe difference between the foothold locations
    # and COM trajectories)
    d1 = cross_mx(W_contacts[util.leg_map["LF"]] - act_state.pose.position)
    d2 = cross_mx(W_contacts[util.leg_map["LH"]] - act_state.pose.position)
    d3 = cross_mx(W_contacts[util.leg_map["RF"]] - act_state.pose.position)
    d4 = cross_mx(W_contacts[util.leg_map["RH"]] - act_state.pose.position)
    # Compute Jb^T
    JbT = np.vstack([np.hstack([np.eye(3), np.eye(3), np.eye(3), np.eye(3)]),
                        np.hstack([d1, d2, d3, d4])])
    #nullify columns relative to legs that are not in contact                                                                                                #
    JbT = JbT.dot(S_mat)

    # Map the total Wrench to grf
    w_des_grf = np.linalg.pinv(JbT, 1e-04).dot(TotWrench)
                
                                                                
    return w_des_grf, Wffwd, Wfbk, Wg

# Whole body controller for HyQ that includes ffd wrench + fb Wrench (Virtual PD) + gravity compensation
#every vector is in the wf                
def QPController(conf, act_state,  des_state, W_contacts,   stance_legs,   params):
    util = Utils()
                
    # compute virtual impedances                
    Wffwd, Wfbk, Wg = computeVirtualImpedanceWrench(conf, act_state, des_state,  W_contacts,  stance_legs,  params) 
    # Total Wrench
    TotWrench =    Wffwd + Wfbk+  Wg
          
    
    #(Ax-b)T*(Ax-b)
    # G = At*A
    # g = -At*b
    #0.5 xT*G*x + gT*x
    #s.t. Cx<=d
        
    # compute cost function
    # Stance matrix
    S_mat = np.diag(np.hstack([stance_legs[util.leg_map["LF"]] * np.ones(3),
                               stance_legs[util.leg_map["LH"]] * np.ones(3),
                               stance_legs[util.leg_map["RF"]] * np.ones(3),
                               stance_legs[util.leg_map["RH"]] * np.ones(3)]))
    # This is a skew symmetric matrix for (xfi-xc)  corressponding  toe difference between the foothold locations
    # and COM/BASE trajectories, this will depend on what what put in act_state)
                         
    d1 = cross_mx(W_contacts[util.leg_map["LF"]] - act_state.pose.position)
    d2 = cross_mx(W_contacts[util.leg_map["LH"]] - act_state.pose.position)
    d3 = cross_mx(W_contacts[util.leg_map["RF"]] - act_state.pose.position)
    d4 = cross_mx(W_contacts[util.leg_map["RH"]] - act_state.pose.position)
    # Compute Jb^T
    JbT = np.vstack([np.hstack([np.eye(3), np.eye(3), np.eye(3), np.eye(3)]),
                        np.hstack([d1, d2, d3, d4])])
    #nullify columns relative to legs that are not in contact                                                                                                #
    JbT = JbT.dot(S_mat)
    
    W =  np.eye(12) * 1e-4           
    G = JbT.T.dot(JbT) + np.eye(12) * 1e-4  #regularize and make it definite positive
    g = -JbT.T.dot(TotWrench)                 
    
    # compute unilateral constraints Cx >= params.f_min => -Cx <= -params.f_min
    C =  - block_diag( params.normals[util.leg_map["LF"]] , 
                       params.normals[util.leg_map["LH"]],
                       params.normals[util.leg_map["RF"]],
                       params.normals[util.leg_map["RH"]]    )                                                 
    #not need to nullify columns relative to legs that are not in contact because the QP solver removes 0 = 0 constraints                                                                                           #
    d = -params.f_min.reshape((4,))
                
    # EXERCISE 11: compute friction cones inequalities A_f x <= 0
    if (params.frictionCones):                                                                
        C_leg = [None]*4            
        for leg in range(4):
            #compute tangential components
            ty = np.cross(params.normals[leg], np.array([1,0,0]))
            tx = np.cross(ty, params.normals[leg])
                                            
            C_leg[leg] = np.array([
                tx  - params.friction_coeff[leg]*params.normals[leg] ,
                -tx - params.friction_coeff[leg]*params.normals[leg] ,
                ty  - params.friction_coeff[leg]*params.normals[leg] ,
                -ty - params.friction_coeff[leg]*params.normals[leg] ])
            
                       
        C =   block_diag( C_leg[util.leg_map["LF"]] , 
                       C_leg[util.leg_map["LH"]],
                           C_leg[util.leg_map["RF"]],
                           C_leg[util.leg_map["RH"]]    )   
        d = np.zeros(C.shape[0]) 
         
    w_des_grf = quadprog_solve_qp(G, g, C, d, None , None)   
 
    #compute constraint violations (take smallest distance from constraint)
    constr_viol = np.zeros(4)
    for leg in range(4): 
       if (params.frictionCones):
            constraintsPerLeg = 4  
       else:
          constraintsPerLeg = 1                                    
       # TODO check this										
       distance_to_violation = np.amin( - C [ constraintsPerLeg*leg : constraintsPerLeg*leg + constraintsPerLeg,:].dot(w_des_grf))  #d should be zeros so does not affect
       constr_viol[leg] = 1/(1.0 + distance_to_violation*distance_to_violation) #goes to 1 when constraints are violated                                
   
    return w_des_grf, Wffwd, Wfbk, Wg, constr_viol
