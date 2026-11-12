
import pinocchio as pin
import numpy as np
from pinocchio.utils import * #rand
from base_controllers.utils.common_functions import getRobotModel


# console print options to see matrix nicely
np.set_printoptions(precision = 3, linewidth = 200, suppress = True)
np.set_printoptions(threshold=np.inf)

# Loading a robot model
robot = getRobotModel('solo', generate_urdf = True)


#start configuration
v  = np.array([0.0   ,  0.0 , 0.0,  0.0,  0.0,       0.0, #underactuated 	
		     0.0,  0.0,  0.0,  0.0,     0.0,  0.0,  0.0,  0.0,  0.0,    0.0,  0.0,  0.0]) #actuated
# set base position
q = pin.neutral(robot.model)
# set joint positions
q[7:] = np.array([-0.2, 0.7, -1.4, -0.2, 0.7, -1.4, -0.2, -0.7, 1.4, -0.2, -0.7, 1.4])

#q[:3] = np.array([1,1,1])
Q = pin.Quaternion(pin.rpy.rpyToMatrix(0.1,0.1,0))
#q[3:3+4] = np.array([Q.x, Q.y, Q.z, Q.w])
#print(q)
# Update the joint and frame placements
pin.forwardKinematics(robot.model,robot.data,q,v)
pin.updateFramePlacements(robot.model,robot.data)
robot.computeAllTerms(q, v) 

M =  pin.crba(robot.model, robot.data, q)
H = pin.nonLinearEffects(robot.model, robot.data, q, v)
G = pin.computeGeneralizedGravity(robot.model,robot.data, q)

#EXERCISE 1: Compute the com of the robot (in WF)
mass_robot = 0
w_com_robot =np.zeros((3))
for idx,name in enumerate(robot.model.joints): 
	if (idx>0): #skip the first universe link	
		mass_link = robot.model.inertias[idx].mass
		mass_robot+= mass_link
		#com_link = robot.data.oMi[idx].act(robot.model.inertias[idx].lever)				
		com_link =  robot.model.inertias[idx].lever		
		w_com_link = robot.data.oMi[idx].rotation.dot(com_link) + robot.data.oMi[idx].translation		
		w_com_robot +=  mass_link * w_com_link
w_com_robot /=mass_robot
print("Com Position w_com_robot: ", w_com_robot)
# compute using native pinocchio function
com_test = pin.centerOfMass(robot.model, robot.data, q, v)
print( "Com Position (pinocchio): ", com_test)

#print(robot.frameJacobian(q,  robot.model.getFrameId('lf_foot'), True,pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  )




# EXERCIZE 2: Compute robot kinetic energy
#get a random generalized velocity 
#v = rand(robot.model.nv)
## Update the joint and frame placements
#pin.forwardKinematics(robot.model,robot.data,q,v)
## compute using generalized velocities and system mass matrix
#EkinRobot = 0.5*v.transpose().dot(M.dot(v))
#
## compute using separate link contributions using spatial algebra
#EKinSystem= 0
##get the spatial velocities of each link
#twists = [f for f  in robot.data.v]
#inertias = [f for f  in robot.model.inertias]
#for idx, inertia in enumerate(inertias):
#	EKinSystem += inertias[idx].vtiv(twists[idx]) # twist x I twist
#EKinSystem *= .5;
#print "TEST1: ", EkinRobot - EKinSystem
## use pinocchio native function
#pin.computeKineticEnergy(robot.model,robot.data,q,v)
#print "TEST2: ", EkinRobot - robot.data.kinetic_energy



##EXERCISE 3: Build the transformation matrix to use com coordinates
# get the location of the base frame
w_base = robot.data.oMi[1].translation
#compute centroidal quantitities (hg, Ag and Ig)
pin.ccrba(robot.model, robot.data, q, v)

#print "Base Position w_base  ", w_base
#G_T_B = np.zeros((robot.model.nv, robot.model.nv))
#G_Tf_B = np.zeros((robot.model.nv, robot.model.nv))
#G_X_B = np.zeros((6, 6))
#
## compute the motion transform from frame B to framge G (G_X_B)
#G_X_B[:3,:3] = np.eye(3)
#G_X_B[3:,3:] = np.eye(3)
#G_X_B[:3,3:] = pin.skew(com_test - w_base)
#
## compute force transform from frame B to framge G (G_Tf_B) 
#G_Xf_B = np.linalg.inv(G_X_B.T)
#
# Couplings from joints on the floating base 
#Mbj = M[:6, 6:]
# Composite rigid body inertia matrix expressed at the CoM: gMb
#gMb = robot.data.Ig
#
## G_M_b^-1 * G_Xf_B * M_bj
#S_G_B = np.linalg.inv(gMb).dot(G_Xf_B.dot(Mbj))
##G_T_B
#G_T_B[:6 , :6] = G_X_B
#G_T_B[6: , 6:] = np.eye(12)
#G_T_B[:6 , 6:] = S_G_B

## double check the transform T has the same propery of the spatial tranforms (G_Tf_B = inv(G_T_B.T))
#G_Tf_B[:6 , :6] = G_Xf_B
#G_Tf_B[6: , 6:] = np.eye(12)
#G_Tf_B[6: , :6] = -(S_G_B.T).dot(G_Xf_B)
#print np.linalg.inv(G_T_B.T) - G_Tf_B

# EXERCISE 4: Check the mass matrix becomes block diagonal if you appy the tranform
#M_g = G_Tf_B * M * np.linalg.inv(G_T_B)
#print "\n The mass matrix expressed at the com becomes diagonal: \n", M_g


# EXERSISE 5: Check that joint gravity vector nullifies
#Grav_W = np.hstack( ( np.array((0.0, 0.0, -9.81)), np.zeros( model.nv - 3)  )).T
#G_g = G_Tf_B.dot(G) #(TODO does not work)
#G_g = -M_g.dot(Grav_W) 

#the fact that all become zero it makes sense if you think the com is not a point solidal with the base link
# but moves with joints, this means that its jacobians has a part from the base and from the joints. 
# so if we try to turn the floating base robot to a fixed base applying the wrench of "God" 
# we will have also an influence on the joint torques that will be cancelled
#print "\n The gravity force vector at the com should be  [0 0 mg   03x1    0nx1 ]: \n", G_g
 