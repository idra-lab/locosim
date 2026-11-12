#common stuff 
import pinocchio as pin
from pinocchio.utils import *
import numpy as np
from numpy import nan
import math
import time as tm
import eigenpy
eigenpy.switchToNumpyMatrix()
import os
from base_controllers.utils.common_functions import *
from base_controllers.utils.ros_publish import RosPub

import L3_conf as conf

#instantiate graphic utils
ros_pub = RosPub("ur5")
robot = getRobotModel("ur5")


# Init variables
zero = np.matrix([0.0, 0.0,0.0, 0.0, 0.0, 0.0]).T
zero_cart = np.matrix([ 0.0, 0.0,0.0]).T
time = 0.0

# Init loggers
q_log = np.empty((6,0))*nan
q_des_log = np.empty((6,0))*nan
qd_log = np.empty((6,0))*nan
qd_des_log = np.empty((6,0))*nan
qdd_log = np.empty((6,0))*nan
qdd_des_log = np.empty((6,0))*nan
x_log = np.empty((3,0))*nan
x_des_log = np.empty((3,0))*nan
xd_log = np.empty((3,0))*nan
xd_des_log = np.empty((3,0))*nan
xdd_log = np.empty((3,0))*nan
xdd_des_log = np.empty((3,0))*nan
euler_log = np.empty((3,0))*nan
euler_des_log = np.empty((3,0))*nan
tau_log = np.empty((6,0))*nan
tauCtrl_log = np.empty((6,0))*nan
tauExt_log = np.empty((6,0))*nan
f_log = np.empty((3,0))*nan
time_log =  np.array([])

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

# Load data from configuration file
q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des  = conf.q0        # joint reference velocity
qd_des = zero        # joint reference acceleration
qdd_des = zero        # joint desired acceleration


# compute initial end effector position and velocity
x0 = robot.framePlacement(q, frame_ee, True).translation + np.matrix([0.0, 0.0, 0.0]).T
xd0 = np.matrix([ 0.0, 0.0, 0.0]).T
xdd0 = np.matrix([ 0.0, 0.0, 0.0]).T
x = x0
xd = xd0
xdd = xdd0
x_des = x0
xd_des = zero_cart
xdd_des = zero_cart
euler = zero_cart
euler_des = zero_cart
extForce = zero_cart
tauExt = zero
tauZero = zero


# CONTROL LOOP
while True:
   

    if time >= conf.exp_duration:
        break
                            
    
    robot.computeAllTerms(q, qd) 
    # joint space inertia matrix                
    M = robot.mass(q, False)
    # bias terms                
    h = robot.nle(q, qd, False)
    #gravity terms                
    g = robot.gravity(q)
    
    #Compute Jacobian of the end-effector (in the WF)
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    # take first 3 rows of J6 cause we have a point contact            
    J = J6[:3,:]
    
    # Moore-penrose pseudoinverse  A^# = (A^TA)^-1 * A^T with A = J^T
    JTpinv = np.linalg.inv(J.dot(J.T)).dot(J)
    
    # compute frame end effector position and velocity in the WF   
    x = robot.framePlacement(q, frame_ee).translation                      
    xd = J.dot(qd)
    
     # ---------------------- DISTURBANCES ---------------------- 
   
    if time > 1.0:
       
        #Torque disturbance        
#        tauExt = np.multiply(conf.ampJS, np.sin(two_pi_f_JS*time + conf.phiJS)) #Sine
#        tauExt = np.matrix([0.0, 0.0, 20.0, 0.0, 0.0, 0.0]).T                   #Step
#        extForce = JTpinv.dot(tauExt)                                                #mapping torques -> forces

    # ---------------------- CONTROLLERS ---------------------- 
                   
     #Exercise 1:  joint-space PD impedance control
#    tau = ...
                    
     #Exercise 2: joint-space PD impedance control + gravity and Coriolis compensation           
#    tau = ...
    
    #Exercise 3: joint-space PD impedance control + gravity and Coriolis compensation + dynamic coupling compensation         
#    tau = ...
    
    #Exercise 4: joint-space PD impedance control + gravity and Coriolis compensation + dynamic coupling compensation + inertia shapping      
#    tau = ...

     
    # ---------------------- SIMULATION -----------------------
     
        #SIMULATION of the forward dynamics    
        M_inv = np.linalg.inv(M)  
        qdd = M_inv*(tau+tauExt-h)    
        
        # Forward Euler Integration    
        qd = qd + qdd*conf.dt
        q = q + qd*conf.dt + 0.5*conf.dt*conf.dt*qdd
        
        # Log Data into a vector
        time_log = np.hstack((time_log, time))                
        q_log = np.hstack((q_log, q ))
        q_des_log= np.hstack((q_des_log, q_des))
        qd_log= np.hstack((qd_log, qd))
        qd_des_log= np.hstack((qd_des_log, qd_des))
        qdd_log= np.hstack((qdd_log, qdd))
        qdd_des_log= np.hstack((qdd_des_log, qdd_des))
        tau_log = np.hstack((tau_log, tau))
    #    tauCtrl_log = np.hstack((tauCtrl_log, tauCtrl))
        tauExt_log = np.hstack((tauExt_log, tauExt))
        f_log = np.hstack((f_log, extForce))
        x_log = np.hstack((x_log, x))
        x_des_log= np.hstack((x_des_log, x_des))
        xd_log= np.hstack((xd_log, xd))
        xd_des_log= np.hstack((xd_des_log, xd_des))
        xdd_log= np.hstack((xdd_log, xdd))
        xdd_des_log= np.hstack((xdd_des_log, xdd_des))
        euler_log= np.hstack((euler_log, euler.reshape(3,-1)))
        euler_des_log= np.hstack((euler_des_log, euler_des.reshape(3,-1)))             
     
        # update time
        time = time + conf.dt                  
                    
        #publish joint variables
        ros_pub.publish(robot, q, qd, tau)                   
        tm.sleep(conf.dt*conf.SLOW_FACTOR)
        
        # stops the while loop if  you prematurely hit CTRL+C                    
        if ros_pub.isShuttingDown():
            print ("Shutting Down")                    
            break;
            
ros_pub.deregister_node()

# plot joint impedances
plotJointImpedance('position', q_log, q_des_log, tauExt_log)
plotJointImpedance('velocity', qd_log, qd_des_log, tauExt_log)
plotJointImpedance('acceleration', qdd_log, qdd_des_log, tauExt_log)







