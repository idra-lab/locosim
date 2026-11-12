import time

from pinocchio.robot_wrapper import RobotWrapper as PinocchioRobotWrapper
from pinocchio.deprecation import deprecated
import pinocchio as pin
import pinocchio.utils as utils
from pinocchio.explog import exp
import numpy as np

class RobotWrapper(PinocchioRobotWrapper):

    
    
    @staticmethod
    def BuildFromURDF(filename, package_dirs=None, root_joint=None, verbose=False, meshLoader=None):
        robot = RobotWrapper()
        robot.initFromURDF(filename, package_dirs, root_joint, verbose, meshLoader)
        #additional var

        # robot mass
        pin.crba(robot.model, robot.data, pin.neutral(robot.model))
        robot.robotMass = robot.data.M[0, 0]
        # TODO: fix in all files (robotMass is prefered)
        robot.robot_mass = robot.robotMass
        #number of attive joints
        if (robot.model.joints[1].nq == 7):
            robot.na = robot.model.nv - 6
        else:
            robot.na = robot.model.nv
        # ee frames, ndeces and names
        robot.getEndEffectorsFrames = []
        robot.getEndEffectorsFrameId = []
        robot.getEndEffectorsFrameNames = []
        for f in robot.model.frames:
            if 'foot' in f.name and 'joint' not in f.name and 'fixed' not in f.name:
                robot.getEndEffectorsFrames.append(f)
                robot.getEndEffectorsFrameId.append(robot.model.getFrameId(f.name))
                robot.getEndEffectorsFrameNames.append(f.name)
        # number of ee
        robot.nee = len(robot.getEndEffectorsFrameId)
        # is floating base
        list_of_joint_types = [i.shortname() for i in robot.model.joints]
        robot.isFloatingBase = pin.JointModelFreeFlyer().shortname() in list_of_joint_types

        return robot
                             
        
    def mass(self, q, update=True):
        if(update):
            return pin.crba(self.model, self.data, q)
        return self.data.M

    def nle(self, q, v, update=True):
        if(update):
            return pin.nonLinearEffects(self.model, self.data, q, v)
        return self.data.nle
        
    def robotComW(self, q=None, v=None, a=None, update=True):
        if(update==False or q is None):
            return PinocchioRobotWrapper.com(self, q);
        if a is None:
            if v is None:
                return PinocchioRobotWrapper.com(self, q)
            return PinocchioRobotWrapper.com(self, q, v)
        return PinocchioRobotWrapper.com(self, q, v,a)

    def robotComB(self, q_j, qd_j=None):
         floating_base_q = pin.neutral(self.model)
         floating_base_q[7:] = q_j
         if qd_j is None:                    
            return PinocchioRobotWrapper.com(self, floating_base_q);
         else:
            floating_base_qd = np.zeros(self.nv)
            floating_base_qd[6:] = qd_j
            return PinocchioRobotWrapper.com(self, floating_base_q, floating_base_qd)
       
    def Jcom(self, q, update=True):
        if(update):
            return pin.jacobianCenterOfMass(self.model, self.data, q)
        return self.data.Jcom

    # centoidal momentum matrix    
    def momentumJacobian(self, q, v, update=True):
        if(update):
            pin.ccrba(self.model, self.data, q, v);
        return self.data.Ag;

    #Robot Centroidal (e.g wrt to com not to base frame origin) inertia in WF (changes with orientation) 
    def centroidalInertiaW(self, q, v, update=True):
        if(update):
            pin.ccrba(self.model, self.data, q, v);
        return self.data.Ig.inertia;

    # Robot Centroidal inertia (e.g wrt to com) in BF (does not changes with orientation)
    def centroidalInertiaB(self, q, v, update=True):
        if(update):
            pin.ccrba(self.model, self.data, q, v);
        return self.data.Ycrb[1].inertia;        
        
        
    #Composite rigid body inertia wrt to the origin of BASE FRAME expressed in BASE FRAME (does not change with orientation)
    def compositeRobotInertiaB(self, q, update=True):
        if(update):
            pin.crba(self.model, self.data, q)
        return self.data.M[3:3+3,3:3+3];


        
    def computeAllTerms(self, q, v):
        ''' pin.computeAllTerms is equivalent to calling:
            pinocchio::forwardKinematics
            pinocchio::crba
            pinocchio::nonLinearEffects
            pinocchio::computeJointJacobians
            pinocchio::centerOfMass
            pinocchio::jacobianCenterOfMass
            pinocchio::kineticEnergy
            pinocchio::potentialEnergy
            This is too much for our needs, so we call only the functions
            we need, including those for the frame kinematics
        '''
#        pin.computeAllTerms(self.model, self.data, q, v);
        #compute first order kinematics								
        pin.forwardKinematics(self.model, self.data, q, v, np.zeros(self.model.nv))
        pin.computeJointJacobians(self.model, self.data)

        pin.crba(self.model, self.data, q)
        pin.nonLinearEffects(self.model, self.data, q, v)
        pin.computeJointJacobiansTimeVariation(self.model, self.data, q, v)
        pin.updateFramePlacements(self.model, self.data)
        
    def forwardKinematics(self, q, v=None, a=None):
        if v is not None:
            if a is not None:
                pin.forwardKinematics(self.model, self.data, q, v, a)
            else:
                pin.forwardKinematics(self.model, self.data, q, v)
        else:
            pin.forwardKinematics(self.model, self.data, q)
               
    def frameJacobian(self, q, index, update=True, ref_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED):
        ''' Call computeFrameJacobian if update is true. If not, user should call computeFrameJacobian first.
            Then call getFrameJacobian and return the Jacobian matrix.
            ref_frame can be: ReferenceFrame.LOCAL, ReferenceFrame.WORLD, ReferenceFrame.LOCAL_WORLD_ALIGNED
        '''
        if(update): 
            pin.computeFrameJacobian(self.model, self.data, q, index)
        return pin.getFrameJacobian(self.model, self.data, index, ref_frame)
        
    def frameVelocity(self, q, v, index, update_kinematics=True, ref_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED):
        if update_kinematics:
            pin.forwardKinematics(self.model, self.data, q, v)
        v_local = pin.getFrameVelocity(self.model, self.data, index)
        v_local = pin.getFrameVelocity(self.model, self.data, index)
        if ref_frame==pin.ReferenceFrame.LOCAL:
            return v_local
            
        H = self.data.oMf[index]
        if ref_frame==pin.ReferenceFrame.WORLD:
            v_world = H.act(v_local)
            return v_world
        
        Hr = pin.SE3(H.rotation, np.zeros(3))
        v = Hr.act(v_local)
        return v
            

    def frameAcceleration(self, q, v, a, index, update_kinematics=True, ref_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED):
        if update_kinematics:
          if a!=None:
              pin.forwardKinematics(self.model, self.data, q, v, a)
          else:
              pin.forwardKinematics(self.model, self.data, q, v)                                        
        a_local = pin.getFrameAcceleration(self.model, self.data, index)
        if ref_frame==pin.ReferenceFrame.LOCAL:
            return a_local
            
        H = self.data.oMf[index]
        if ref_frame==pin.ReferenceFrame.WORLD:
            a_world = H.act(a_local)
            return a_world
        
        Hr = pin.SE3(H.rotation, np.zeros(3))
        a = Hr.act(a_local)
        return a
        
    def frameClassicAcceleration(self, q, v, a, index, update_kinematics=True, ref_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED):
                   
        if update_kinematics:
          if a!=None:
              pin.forwardKinematics(self.model, self.data, q, v, a)
          else:
              pin.forwardKinematics(self.model, self.data, q, v)    
        v = pin.getFrameVelocity(self.model, self.data, index)
        a_local = pin.getFrameAcceleration(self.model, self.data, index)
        a_local.linear += np.cross(v.angular, v.linear, axis=0)
        if ref_frame==pin.ReferenceFrame.LOCAL:
            return a_local
            
        H = self.data.oMf[index]
        if ref_frame==pin.ReferenceFrame.WORLD:
            a_world = H.act(a_local)
            return a_world
        #LOCAL_WORLD_ALIGNED is the horizontal frame
        Hr = pin.SE3(H.rotation, np.zeros(3))
        a = Hr.act(a_local)
        return a
      
    def deactivateCollisionPairs(self, collision_pair_indexes):
        for i in collision_pair_indexes:
            self.collision_data.deactivateCollisionPair(i);
            
    def addAllCollisionPairs(self):
        self.collision_model.addAllCollisionPairs();
        self.collision_data = pin.GeometryData(self.collision_model);
        
    def isInCollision(self, q, stop_at_first_collision=True):
        return pin.computeCollisions(self.model, self.data, self.collision_model, self.collision_data, np.asmatrix(q).reshape((self.model.nq,1)), stop_at_first_collision);

    def findFirstCollisionPair(self, consider_only_active_collision_pairs=True):
        for i in range(len(self.collision_model.collisionPairs)):
            if(not consider_only_active_collision_pairs or self.collision_data.activeCollisionPairs[i]):
                if(pin.computeCollision(self.collision_model, self.collision_data, i)):
                    return (i, self.collision_model.collisionPairs[i]);
        return None;
        
    def findAllCollisionPairs(self, consider_only_active_collision_pairs=True):
        res = [];
        for i in range(len(self.collision_model.collisionPairs)):
            if(not consider_only_active_collision_pairs or self.collision_data.activeCollisionPairs[i]):
                if(pin.computeCollision(self.collision_model, self.collision_data, i)):
                    res += [(i, self.collision_model.collisionPairs[i])];
        return res

    def getEEStackJacobians(self, q, component='full', ref_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED):
        ee_idxs = self.getEndEffectorsFrameId
        nee = len(ee_idxs)
        if component == 'full':
            nc = 6
            startJc = 0
            stopJc = 6

        elif component == 'linear':
            nc = 3
            startJc = 0
            stopJc = 3

        elif component == 'angular':
            nc = 3
            startJc = 3
            stopJc = 6

        Jc = np.zeros([nc * nee, self.nv])
        for i in range(0, nee):
            idx = ee_idxs[i]
            Jc[nc * i:nc * (i + 1), :] = pin.computeFrameJacobian(self.model, self.data, q, idx, ref_frame)[
                                         startJc:stopJc]
        return Jc

    def KKTMatrixAtEndEffectors(self, q, component='full', ref_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED):
        M = self.mass(q)
        ee_idxs = self.getEndEffectorsFrameId
        nee = len(ee_idxs)
        if nee == 0:
            return M
        Jc = self.getEEStackJacobians(q, component, ref_frame)

        zeros = np.zeros([Jc.shape[0], Jc.shape[0]])

        raw0 = np.hstack([M, Jc.transpose()])
        raw1 = np.hstack([Jc, zeros])
        KKTMatrix = np.vstack([raw0, raw1])
        return KKTMatrix

    def KKTMatrixAtEndEffectorsInv(self, q, component='full', ref_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED):
        Jc = self.getEEStackJacobians(q, component, ref_frame)
        return pin.computeKKTContactDynamicMatrixInverse(self.model, self.data, q, Jc)

    def dJdq(self, q, v, index, component='full', update_kinematics=True,
             ref_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED):
        frame_acc = self.frameClassicAcceleration(q, v, None, index, update_kinematics, ref_frame)
        if component == 'full':
            res = np.hstack([frame_acc.linear, frame_acc.angular])
        elif component == 'linear':
            res = frame_acc.linear
        elif component == 'angular':
            res = frame_acc.angular
        return res

    def frictionRegressor(self, v):
        # Fv: stack of viscous friction coefficients
        # Fc: stack of Coulomb friction coefficients
        # Fo: stack of friction offsets
        #                                                 [Fv]
        # tau_friction[i] = frictionRegressor(qd)[i, :] * [Fc]
        #

        #Y_v = np.diag(v[6:])
        Y_c = np.diag(np.sign(v[6:]))
        #Y_o = np.eye(self.na)

        #Y_f = np.hstack([Y_v, Y_c])#, Y_o])

        return Y_c
__all__ = ['RobotWrapper']


