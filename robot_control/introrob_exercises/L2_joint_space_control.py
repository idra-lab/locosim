#common stuff
from __future__ import print_function
import pinocchio as pin
from numpy import nan
import math
import time as tm

from base_controllers.utils.common_functions import *
from base_controllers.utils.ros_publish import RosPub

import L2_conf as conf

#instantiate graphic utils
os.system("killall rosmaster rviz")
ros_pub = RosPub("ur5")
robot = getRobotModel("ur5")


# Init variables
zero = np.array([0.0, 0.0,0.0, 0.0, 0.0, 0.0])
time = 0.0

two_pi_f             = 2*np.pi*conf.freq   # frequency (time 2 PI)
two_pi_f_amp         = np.multiply(two_pi_f, conf.amp) 
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

# Init loggers
buffer_size = int(math.floor(conf.exp_duration/conf.dt))
q_log = np.empty((6, buffer_size))*nan
q_des_log = np.empty((6, buffer_size))*nan
qd_log = np.empty((6, buffer_size))*nan
qd_des_log = np.empty((6, buffer_size))*nan
qdd_log = np.empty((6, buffer_size))*nan
qdd_des_log = np.empty((6, buffer_size))*nan
tau_log = np.empty((6, buffer_size))*nan
f_log = np.empty((3,buffer_size))*nan
p_log = np.empty((3, buffer_size))*nan
time_log =  np.empty((buffer_size))*nan
log_counter = 0

# EXERCISE 2.2: 
#conf.qd0 = two_pi_f_amp

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des  = conf.q0        # joint reference velocity
qd_des = zero        # joint reference acceleration
qdd_des = zero        # joint desired acceleration

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)
qd_des_old = zero
q_des_old = conf.q0 

contact_sampled = False

# CONTROL LOOP
while True:
    
    # EXERCISE 1.1: Sinusoidal reference Generation
    q_des  = conf.q0 +   np.multiply( conf.amp, np.sin(two_pi_f*time + conf.phi))
    qd_des = np.multiply(two_pi_f_amp , np.cos(two_pi_f*time + conf.phi))
    qdd_des = np.multiply( two_pi_f_squared_amp , -np.sin(two_pi_f*time + conf.phi))

    # Set constant reference after a while
    if time >= conf.exp_duration_sin:
        q_des  = conf.q0
        qd_des=zero
        qdd_des=zero          

    # EXERCISE 1.2: Step reference Generation
    # if time > 2.0:
    #     q_des = conf.q0 + np.array([ 0.0, -0.4, 0.0, 0.0,  0.5, 0.0])
    #     qd_des =  zero
    #     qdd_des = zero
    # else:
    #     q_des = conf.q0
    #     qd_des =  zero
    #     qdd_des = zero
        
    # EXERCISE 2.4: Constant reference
    # q_des = conf.q0
    # qd_des =  zero
    # qdd_des = zero

    # Decimate print of time
    #if (divmod(time ,1.0)[1]  == 0):
       #print('Time %.3f s'%(time))
    if time >= conf.exp_duration:
        break
                            
    robot.computeAllTerms(q, qd) 
    # joint space inertia matrix                
    M = robot.mass(q)
    # bias terms                
    h = robot.nle(q, qd)
    #gravity terms                
    g = robot.gravity(q)
    
    #compute ee position  in the world frame  
    p = robot.framePlacement(q, frame_ee).translation 
    # compute jacobian of the end effector in the world frame  
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    # take first 3 rows of J6 cause we have a point contact            
    J = J6[:3,:] 

        
    # EXERCISE  1.5: PD control - critical damping
    # conf.kd[0,0] = 2*np.sqrt(conf.kp[0,0]*M[0,0])
    # conf.kd[1,1] = 2*np.sqrt(conf.kp[1,1]*M[1,1])
    # conf.kd[2,2] = 2*np.sqrt(conf.kp[2,2]*M[2,2])
    # conf.kd[3,3] = 2*np.sqrt(conf.kp[3,3]*M[3,3])
    # conf.kd[4,4] = 2*np.sqrt(conf.kp[4,4]*M[4,4])
    # conf.kd[5,5] = 2*np.sqrt(conf.kp[5,5]*M[5,5])
    # print (conf.kd[1,1])
    # print (conf.kd[4,4])

    # EXERCISE  2.3: Inverse Dynamics (computed torque) - low gains
#    conf.kp = np.eye(6)*60
#    conf.kd[0,0] = 2*np.sqrt(conf.kp[0,0])
#    conf.kd[1,1] = 2*np.sqrt(conf.kp[1,1])
#    conf.kd[2,2] = 2*np.sqrt(conf.kp[2,2])
#    conf.kd[3,3] = 2*np.sqrt(conf.kp[3,3])
#    conf.kd[4,4] = 2*np.sqrt(conf.kp[4,4])
#    conf.kd[5,5] = 2*np.sqrt(conf.kp[5,5])
                                       
    #CONTROLLERS                                    
    #Exercise 1.3:  PD control
    tau = conf.kp.dot(q_des-q) + conf.kd.dot(qd_des-qd)
    
    # Exercise 1.6: PD control + Gravity Compensation
    #tau = conf.kp.dot(q_des-q) + conf.kd.dot(qd_des-qd)  + g
    
    # Exercise 1.7: PD + gravity + Feed-Forward term   
    #M_des = robot.mass(q_des)
    #tau =  np.multiply(np.diag(M_des), qdd_des) + conf.kp.dot(q_des-q) + conf.kd.dot(qd_des-qd)  + g
 
    # EXERCISE 2.1 Inverse Dynamics (Computed Torque)
    #tau = M.dot(qdd_des + conf.kp.dot(q_des-q) + conf.kd.dot(qd_des-qd)) + h

    # EXERCISE 2.5 Inverse Dynamics (Computed Torque) - uncertainty in the cancellation   
    # M_hat  = M*1.1
    # h_hat  = h*1.1
    # tau = M_hat.dot(qdd_des + conf.kp.dot(q_des-q) + conf.kd.dot(qd_des-qd)) + h_hat

    # EXERCISE 2.6  Inverse Dynamics (Desired states)
    # M_des = robot.mass(q_des)
    # h_des = robot.nle(q_des, qd_des)
    # tau = M_des.dot(qdd_des + conf.kp.dot(q_des-q) + conf.kd.dot(qd_des-qd)) + h_des
    
    if conf.EXTERNAL_FORCE:   		
        # EXERCISE 2.4: Add external force at T = 3.0s
        if time>3.0:
           F_env = conf.extForce
        else:
           F_env = np.array([0.0, 0.0, 0.0])
        			
        # EXERCISE 2.7: Add  unilateral compliant contact (normal spring)
        # pd = J.dot(qd)
        # if (conf.n.dot(conf.p0 - p)>0.0):
        #     Fk = conf.K_env.dot(conf.p0 - p)
        #     if (conf.n.dot(pd)<0.0):
        #         Fd = - conf.D_env.dot(pd)
        #     F_proj =  np.dot(conf.n, Fk + Fd) #scalar
        #     F_env = conf.n * F_proj #vector
        # else:
        #     F_env = np.array([0.0, 0.0, 0.0])
        #
#       # EXERCISE 2.8: Add  unilateral compliant contact  (full 3D model)
#         pd = J.dot(qd)
#         if (conf.n.dot(conf.p0 - p)>0.0):
#             # sample P0
#             if (not contact_sampled):
#                 conf.p0 = p
#                 contact_sampled = True
#             F_env =  conf.K_env.dot(conf.p0 - p)
#             if (conf.n.dot(pd) < 0.0):
#                 F_env += - conf.D_env.dot(pd)
#
#            # EXERCISE 2.9: Friction coefficient
#             # # clamp to friction cone
#             # # X component
#             # if (F_env[0] >= conf.mu * F_env[2]):
#             #     F_env[0] = conf.mu * F_env[2]
#             # if (F_env[0] <= -conf.mu * F_env[2]):
#             #     F_env[0] = -conf.mu * F_env[2]
#             # # Y component
#             # if (F_env[1] >= conf.mu * F_env[2]):
#             #     F_env[1] = conf.mu * F_env[2]
#             # if (F_env[1] <= -conf.mu * F_env[2]):
#             #     F_env[1] = -conf.mu * F_env[2]
#             # ros_pub.add_cone(p, conf.n, conf.mu, color = "blue")
#
#         else:
#             contact_sampled = False
#             F_env = np.array([0.0, 0.0, 0.0])


        ros_pub.add_arrow(p,F_env/100.)
        tau += J.transpose().dot(F_env)
    ros_pub.add_marker(p)     				
    		      
  
    #SIMULATION of the forward dynamics    
    M_inv = np.linalg.inv(M)  
    qdd = M_inv.dot(tau-h)    
    
    # Forward Euler Integration    
    qd = qd + qdd*conf.dt    
    q = q + conf.dt*qd  + 0.5*conf.dt*conf.dt*qdd
    

    # Log Data into a vector
    time_log[log_counter] = time
    q_log[:,log_counter] = q
    q_des_log[:,log_counter] = q_des
    qd_log[:,log_counter] = qd
    qd_des_log[:,log_counter] = qd_des
    qdd_log[:,log_counter] = qdd
    qdd_des_log[:,log_counter] = qdd_des
    p_log[:, log_counter] = p
    tau_log[:,log_counter] = tau
    log_counter+=1
         
 
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
        
                 
                
# plot joint variables                                                                              
plotJoint('position', time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('velocity', time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('acceleration', time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
#plotJoint('torque', time_log, q_log, q_des_log, qd_log, qd_des_log, qdd_log, qdd_des_log, tau_log)
plotEndeff('position', 5, time_log, p_log)





