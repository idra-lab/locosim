#common stuff
from __future__ import print_function
import pinocchio as pin
from pinocchio.utils import *
from numpy import nan
import math
import time as tm
from base_controllers.utils.common_functions import *
from base_controllers.utils.ros_publish import RosPub
from base_controllers.utils.math_tools import Math
import L3_conf as conf

#instantiate graphic utils
os.system("killall rosmaster rviz")
ros_pub = RosPub("ur5")
robot = getRobotModel("ur5")

math_utils = Math()
# Init variables
zero = np.array([0.0, 0.0,0.0, 0.0, 0.0, 0.0])
zero_cart = np.array([ 0.0, 0.0,0.0])
time = 0.0

two_pi_f             = 2*np.pi*conf.freq   # frequency (time 2 PI)
two_pi_f_amp         = np.multiply(two_pi_f, conf.amp) 
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

# Init loggers
buffer_size = int(math.floor(conf.exp_duration/conf.dt))
log_counter = 0
p_log = np.empty((3, buffer_size))*nan
p_des_log = np.empty((3,buffer_size))*nan
pd_log = np.empty((3,buffer_size))*nan
pd_des_log = np.empty((3,buffer_size))*nan
pdd_des_log = np.empty((3,buffer_size))*nan
rpy_log = np.empty((3,buffer_size))*nan
rpy_des_log = np.empty((3,buffer_size))*nan
error_o_log = np.empty((3,buffer_size))*nan
tau_log = np.empty((6,buffer_size))*nan
time_log =  np.empty((buffer_size))*nan

rpy_old = np.zeros((3))
rpy_unwrapped = np.zeros((3))
rpy_des_old = np.zeros((3))
rpy_des_unwrapped = np.zeros((3))

q = conf.q0
qd = conf.qd0
qdd = conf.qdd0

q_des  = conf.q0     # joint reference velocity
qd_des = zero        # joint reference acceleration
qdd_des = zero       # joint reference acceleration

# get the ID corresponding to the frame we want to control
assert(robot.model.existFrame(conf.frame_name))
frame_ee = robot.model.getFrameId(conf.frame_name)

# Compute initial end effector position and velocity from q0
p0 = robot.framePlacement(conf.q0, frame_ee, True).translation + np.array([0.0, 0.0, 0.0])
pd0 = np.array([ 0.0, 0.0, 0.0])
pdd0 = np.array([ 0.0, 0.0, 0.0])

# initialize actual variables
p = p0
pd = pd0
pdd = pdd0
rpy = zero_cart
# initialize reference variables
p_des = p0
pd_des = zero_cart
pdd_des = zero_cart
rpy_des = zero_cart

FirstTime = True

# CONTROL LOOP
while True:
    
    # EXE  1.1: Sinusoidal reference generation for the end-effector
    p_des  = p0  + np.multiply(conf.amp, np.sin(two_pi_f*time + conf.phi))
    pd_des = np.multiply(two_pi_f_amp , np.cos(two_pi_f*time + conf.phi))
    pdd_des = np.multiply(two_pi_f_squared_amp , -np.sin(two_pi_f*time + conf.phi))
    # Set constant reference after a while
    if time >= conf.exp_duration_sin:
        p_des  = p0
        pd_des = pd0
        pdd_des = pdd0
        
    #  EXE  1.2: Step reference generation for the end effector
    # if time > 2.0:
    #     p_des = p0 + np.array([ 0.0, 0.0, 0.1])
    #     pd_des =  zero_cart
    #     pdd_des = zero_cart
    # else:
    #     p_des = p0
    #     pd_des =  zero_cart
    #     pdd_des = zero_cart

#    EXE  2.3: Constant reference
#     p_des = p0
#     pd_des =  zero_cart
#     pdd_des = zero_cart

    # Decimate print of time
    #if (divmod(time ,1.0)[1]  == 0):
       #print('Time %.3f s'%(time))
    if time >= conf.exp_duration:
        break
                            
    robot.computeAllTerms(q, qd) 
    # joint space inertia matrix                
    M = robot.mass(q, False)
    # bias terms                
    h = robot.nle(q, qd, False)
    #gravity terms                
    g = robot.gravity(q)
    
    # compute the Jacobian of the end-effector in the world frame
    J6 = robot.frameJacobian(q, frame_ee, False, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)                    
    # take first 3 rows of J6 cause we have a point contact            
    J = J6[:3,:] 
    # compute  the end-effector acceleration due to joint velocity Jdot*qd         
    Jdqd = robot.frameClassicAcceleration(q, qd, None, frame_ee).linear    
    # compute frame end effector position and velocity in the WF   
    p = robot.framePlacement(q, frame_ee).translation  

    # with sine reference: to avoid having tracking errors in velocity at the initial point (only runs first time)
    # if conf.RemoveInitialError:
    #     initial_pd = two_pi_f_amp
    #     initial_rpyd = np.array([2.2*np.pi, 0.0 , 0.0])
    #     initial_omega = math_utils.Tomega(np.zeros(3)).dot(initial_rpyd)
    #     qd = J6.dot(np.hstack((initial_pd,  initial_omega )) )
    #     conf.RemoveInitialError = False
                    
    pd = J.dot(qd)  

    M_inv = np.linalg.inv(M)
    # Moore-penrose pseudoinverse  A^# = (A^T*A)^-1 * A^T with A = J^T
    JTpinv = np.linalg.inv(J.dot(J.T)).dot(J)
    
    # joint space inertia matrix reflected at the end effector (J*M^-1*Jt)^-1
    lambda_= np.linalg.inv(J.dot(M_inv).dot(J.T))  # J should be full row rank  otherwise add a damping term
     
    #Null space projector I - (JTpinv )^-1 * JTpinv => I  - JT *JTpiv
    N = eye(6)-J.T.dot(JTpinv)

    # EXE 1.4: PD control (cartesian task)
    F_des = conf.Kx.dot(p_des-p) + conf.Dx.dot(pd_des-pd)
    tau = J.T.dot(F_des)

    # EXE 1.5: PD control (cartesian task) + postural task
    # null space torques (postural task)
    tau0 = conf.Kp_postural*(conf.q0-q) - conf.Kd_postural*qd
    tau_null = N.dot(tau0)
    tau = J.T.dot(F_des)  + tau_null
    
    # EXE 1.6: PD control + Gravity Compensation:
    F_des = conf.Kx.dot(p_des-p) + conf.Dx.dot(pd_des-pd)	+ JTpinv.dot(g)
    tau = J.T.dot(F_des) + tau_null
        
        
    # EXE 1.7: PD control  + Gravity Compensation + Feed-Forward term
    F_des = lambda_.dot(pdd_des) + conf.Kx.dot(p_des-p)+conf.Dx.dot(pd_des-pd) + JTpinv.dot(g)
    tau = J.T.dot(F_des) + tau_null
     
     
    # EXE 2.1: Cartesian space inverse dynamics
    F_des = pdd_des + conf.Kx.dot(p_des-p) + conf.Dx.dot(pd_des-pd)
    mu =  JTpinv.dot(h) -lambda_.dot(Jdqd)
    tau = J.T.dot(lambda_.dot(F_des) + mu) + tau_null
    
    
    # EXE 2.2: Cartesian space inverse dynamics with bias compensation in joint space (simpler to compute)
    # F_des = pdd_des + conf.Kx.dot(p_des-p) + conf.Dx.dot(pd_des-pd)
    tau =  J.T.dot(lambda_.dot(F_des)) + h + tau_null


    # EXE 3.1:  Control of orientation with PD: constant orientation
    # ORIENTATION_CONTROL = True
    # # get the actual end-effector orientation (columns are the axis of frame_ee expressed in WF (check rviz TF) )
    # w_R_e = robot.framePlacement(q, frame_ee).rotation
    # # extract actual euler angles for logging
    # rpy = math_utils.rot2eul(w_R_e)
    # # compute the actual end-effector twist (i.e. linear and angular velocity)
    # twist = J6.dot(qd)
    # # extract omega
    # omega = twist[3:6]

    # compose the des orientation rotation matrix 180deg about x (x axis along x, y axis along -y, z axis along -z) NB the axis are the columns of the matrix w_R_des
    # des_x_axis = np.array([1.,  0.,  0.])
    # des_y_axis = np.array([0., -1.,  0.])
    # des_z_axis = np.array([0.,  0., -1.])
    # w_R_des = np.vstack((des_x_axis.T, des_y_axis.T, des_z_axis.T))
    # omega_des = np.array([0.,0.,0.])

    # EXE 3.2 - Control of orientation with PD: singularity with Euler Angles
    # rpy_des = np.array([0.8, math.pi/2 , 0.0]) # singularity
    # rpy_des = np.array([0.2, 0.4, 0.8])  # random orient with Euler Angles
    # w_R_des = math_utils.eul2Rot(rpy_des)


    # EXE 3.3: Control of orientation with PD - sinusoidal reference
    # rpy0 = math_utils.rot2eul(robot.framePlacement(conf.q0, frame_ee).rotation)
    # rpy_des =  rpy0 + np.array([2.2*np.pi*np.sin(time), 0.0 , 0.0])
    # rpyd_des = np.array([2.2*np.pi*np.cos(time), 0.0 , 0.0])
    # rpydd_des = np.array([-2.2*np.pi*np.sin(time), 0.0 , 0.0])
    # # compute rotation matrix representing the desired orientation from Euler Angles
    # w_R_des = math_utils.eul2Rot(rpy_des)
    # # desired angular velocity
    # omega_des = math_utils.Tomega(rpy_des).dot(rpyd_des)
    # # desired angular acceleration
    # omega_d_des = math_utils.Tomega(rpy_des).dot(rpydd_des) +  math_utils.Tomega_dot(rpy_des, rpyd_des).dot(rpyd_des)
    #
   
    # EXE 3.1: Control of orientation with PD
#     # compute rotation matrix from actual orientation of ee to the desired
#     e_R_des = w_R_e.T.dot(w_R_des)
# #    # compute the angle-axis representation of the associated orientation error
#     # compute the angle: method 1) with arc cos
#     arg = (e_R_des[0,0]+ e_R_des[1,1]+ e_R_des[2,2]-1)/2
#     delta_theta = np.arccos(arg)
#     # compute the angle: method 2) with atan2
#     # delta_theta = math.atan2(np.sqrt(pow(e_R_des[2,1]-e_R_des[1,2], 2) +  pow(e_R_des[0,2]-e_R_des[2,0], 2) + pow(e_R_des[1,0]-e_R_des[0,1], 2)), e_R_des[0,0]+ e_R_des[1,1]+ e_R_des[2,2]-1 )
#     # compute the axis (deal with singularity)
#     if delta_theta == 0.0:
#         e_error_o = np.zeros(3)
#     else:
#         r_hat = 1/(2*np.sin(delta_theta))*np.array([e_R_des[2,1]-e_R_des[1,2], e_R_des[0,2]-e_R_des[2,0], e_R_des[1,0]-e_R_des[0,1]])
#         # compute the orientation error
#         e_error_o = delta_theta * r_hat
# #    # the error is expressed in the end-effector frame
# #    # we need to map it in the world frame to compute the moment because the jacobian is in the WF
#     w_error_o = w_R_e.dot(e_error_o)
# #    # Compute the virtual force (linear part of the wrench)
#     F_des = conf.Kx.dot(p_des-p) + conf.Dx.dot(pd_des-pd)
#     # compute the virtual moment (angular part of the wrench) to realize the orientation task
#     Gamma_des =  conf.Ko.dot(w_error_o) + conf.Do.dot(omega_des - omega)
# #    # stack the previously computed linear part with the angular part
#     W_des = np.hstack([F_des, Gamma_des])
# #    # map to torques
#     tau = J6.T.dot(W_des)  + g




    # EXERCISE 3.4: Control of orientation with PD - unwrapping  
    #unwrap euler angles  (both desired and actual)
    UNWRAPPPING = True
    for i in range(3):
        rpy_unwrapped[i] = rpy[i];
        while (rpy_unwrapped[i] < rpy_old[i]  - math.pi):
            rpy_unwrapped[i] += 2*math.pi
        while (rpy_unwrapped[i] > rpy_old[i]  + math.pi):
            rpy_unwrapped[i] -= 2*math.pi
        rpy_old[i] = rpy_unwrapped[i]
    for i in range(3):
        rpy_des_unwrapped[i] = rpy_des[i];
        while (rpy_des_unwrapped[i] < rpy_des_old[i]  - math.pi):
            rpy_des_unwrapped[i] += 2*math.pi
        while (rpy_des_unwrapped[i] > rpy_des_old[i]  + math.pi):
            rpy_des_unwrapped[i] -= 2*math.pi
        rpy_des_old[i] = rpy_des_unwrapped[i]
    
    #EXE  3.6 : full task space inverse dynamics (computed torque)
    # # compute lambda for both orientation and position
    # rho = 0.000001 # damping factor
    # # print("J6 rank:", np.linalg.matrix_rank(J6))
    # # if (np.linalg.matrix_rank(J6) < 6):
    # #     print(J6)
    # #lambda6= np.linalg.inv(J6.dot(M_inv).dot( J6.T))  #singular because J6 not full rank at q0
    # lambda6 = np.linalg.inv(J6.dot(M_inv).dot( J6.T) + pow(rho,2)*eye(6)) # damped inertia matrix
    # #J6Tpinv = np.linalg.pinv(J6.T, rho) # damped pseudoinverse using native function
    # J6Tpinv = np.linalg.inv(J6.dot(J6.T) + pow(rho,2)*eye(6)).dot(J6)  # damped pseudoinverse explicitely computed
    # Jdqd6 = robot.frameClassicAcceleration(q, qd, None, frame_ee).vector
    # mu6 =  J6Tpinv.dot(h) -lambda6.dot(Jdqd6)
    # tau = J6.T.dot(lambda6.dot(np.hstack((pdd_des + F_des, omega_d_des + Gamma_des))) + mu6)
    			
#    EXE  2.3: Add an external force
    if conf.EXTERNAL_FORCE  and time>1.0:
        tau += J.transpose().dot(conf.extForce)
        ros_pub.add_arrow(p, conf.extForce/100)                    
    
    #SIMULATION of the forward dynamics
    qdd = M_inv.dot(tau - h)    
    
    # Forward Euler Integration    
    qd = qd + qdd*conf.dt
    q = q + qd*conf.dt + 0.5*conf.dt*conf.dt*qdd
				    
    # Log Data into a vector
    time_log[log_counter] = time
    p_log[:,log_counter] = p
    p_des_log[:,log_counter] = p_des
    pd_log[:,log_counter] = pd
    pd_des_log[:,log_counter] = pd_des
    tau_log[:,log_counter] = tau

    try: 
        UNWRAPPPING
        rpy_log[:,log_counter] = rpy_unwrapped
        rpy_des_log[:,log_counter] = rpy_des_unwrapped
    except:    
        rpy_log[:,log_counter] = rpy
        rpy_des_log[:,log_counter] = rpy_des

    try: 
        ORIENTATION_CONTROL
        error_o_log[:,log_counter] =  w_error_o
    except: 
        pass                      
 
    log_counter+=1  
    # update time
    time = time + conf.dt                  
    
    # plot ball at the end-effector
    ros_pub.add_marker(p)
    ros_pub.add_marker(p_des, color="blue")
    # publish joint variables
    ros_pub.publish(robot, q, qd, tau)                   
    tm.sleep(conf.dt*conf.SLOW_FACTOR)
    
    # stops the while loop if  you prematurely hit CTRL+C                    
    if ros_pub.isShuttingDown():
        print ("Shutting Down")                    
        break;            
ros_pub.deregister_node()
      
#plot position
plotEndeff('ee position', 1,time_log, p_log, p_des_log)
#plotEndeff('velocity', 2, time_log, p_log, p_des_log, pd_log, pd_des_log, rpy_log, rpy_des_log)
try:
    ORIENTATION_CONTROL
    plotEndeff('euler angles', 3,time_log, rpy_log, rpy_des_log)
    plotEndeff('orientation error', 4,time_log, error_o_log)
except: 
    pass   
plt.show(block=True)