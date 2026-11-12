# Description
# File contains some necessary control algorithms for HyQ
# Author: Michele Focchi
# Date: 23-10-2022
import numpy as np
from base_controllers.utils.optimTools import quadprog_solve_qp
from base_controllers.utils.utils import Utils
from  scipy.linalg import block_diag
import pinocchio as pin
import time

class WholeBodyController():
    def __init__(self, robot_params, real_robot = False, robot = None):
        self.robot = robot
        self.robot_params = robot_params
        self.real_robot = real_robot
        # load gains
        if self.real_robot:
            real_str = '_real'
        else:
            real_str = ''

        self.u = Utils()

        # virtual impedance wrench control
        self.kp_lin = np.diag(self.robot_params.get('kp_lin' + real_str, np.zeros(3)))
        self.kd_lin = np.diag(self.robot_params.get('kd_lin' + real_str, np.zeros(3)))

        self.kp_ang = np.diag(self.robot_params.get('kp_ang' + real_str, np.zeros(3)))
        self.kd_ang = np.diag(self.robot_params.get('kd_ang' + real_str, np.zeros(3)))

        # updated in WBC
        self.kp_linW = np.zeros_like(self.kp_lin)
        self.kd_linW = np.zeros_like(self.kd_lin)

        self.kp_angW = np.zeros_like(self.kp_ang)
        self.kd_angW = np.zeros_like(self.kd_ang)


        self.g_mag = np.linalg.norm(self.robot.model.gravity.vector)


        self.wrench_fbW = np.zeros(6)
        self.wrench_ffW = np.zeros(6)
        self.wrench_gW = np.zeros(6)
        self.wrench_gW[self.u.sp_crd["LZ"]] = self.robot.robotMass * self.g_mag
        self.wrench_desW = np.zeros(6)

        self.wrench_fbW_log = np.full((6, self.robot_params['buffer_size']), np.nan)
        self.wrench_ffW_log = np.full((6, self.robot_params['buffer_size']), np.nan)
        self.wrench_gW_log = np.full((6, self.robot_params['buffer_size']), np.nan)
        self.wrench_desW_log = np.full((6, self.robot_params['buffer_size']), np.nan)

        self.NEMatrix = np.zeros([6, 3 * len(self.robot_params['ee_frames'])])  # Newton-Euler matrix

    def setGains(self, kp_lin, kd_lin, kp_ang, kd_ang):
        self.kp_lin = np.diag(kp_lin)
        self.kd_lin = np.diag(kd_lin)
        self.kp_ang = np.diag(kp_ang)
        self.kd_ang = np.diag(kd_ang)

    def logData(self, log_counter):
        self.wrench_fbW_log[:, log_counter] = self.wrench_fbW
        self.wrench_ffW_log[:, log_counter] = self.wrench_ffW
        self.wrench_gW_log[:, log_counter] = self.wrench_gW
        self.wrench_desW_log[:, log_counter] = self.wrench_desW

    def gravityCompensation(self, W_contacts, wJ, h_joints, basePoseW, comPoseW, comControlled = False):
        # require the call to updateKinematics
        return self.computeWBC(W_contacts, wJ, h_joints, basePoseW, comPoseW, baseTwistW = np.zeros(6), comTwistW= np.zeros(6), des_pose = None, des_twist = None, des_acc = None, comControlled = comControlled, type = 'projection')

    def gravityCompensationBase(self, B_contacts, wJ, h_joints, basePoseW, stance_legs=[True, True, True, True], numberOfJointsPerLeg=3):
        self.wrench_desW = np.zeros(6)
        self.wrench_desW[self.u.sp_crd["LZ"]] = self.robot.robotMass * self.g_mag
        w_R_b = pin.rpy.rpyToMatrix(self.u.angPart(basePoseW))
        # wrench = NEMatrix @ grfs
        for leg in range(len(self.robot_params['ee_frames'])):
            start_col = 3 * leg
            end_col = 3 * (leg + 1)
            if stance_legs[leg]:  # self.contact_state[leg]:
                # ---> linear part
                # identity matrix (I avoid to rewrite zeros)
                self.NEMatrix[self.u.sp_crd["LX"], start_col] = 1.
                self.NEMatrix[self.u.sp_crd["LY"], start_col + 1] = 1.
                self.NEMatrix[self.u.sp_crd["LZ"], start_col + 2] = 1.
                # ---> angular part
                # all in a function
                self.NEMatrix[self.u.sp_crd["AX"]:self.u.sp_crd["AZ"] + 1, start_col:end_col] =  pin.skew(w_R_b.dot(B_contacts[leg]))
            else:
                # clean the matrix (where there are zeros the grf will be zero and so the torques)
                self.NEMatrix[:, start_col:end_col] = 0.

        grForcesW_wbc = np.zeros(self.robot.na)

        grForcesW_wbc = self.projectionWBC()

        tau_ffwd = np.zeros(self.robot.na)

        for leg in range(
                4):  # (where there are zeros in the Matrix, the grf will be zero and so the torques, no need to check stance legs here)
            tau_leg = self.u.getLegJointState(leg, h_joints,numberOfJointsPerLeg=numberOfJointsPerLeg) - \
                      wJ[leg].T @ self.u.getLegJointState(leg, grForcesW_wbc,numberOfJointsPerLeg=numberOfJointsPerLeg)
            self.u.setLegJointState(leg, tau_leg, tau_ffwd, numberOfJointsPerLeg=numberOfJointsPerLeg)

        return tau_ffwd, grForcesW_wbc



    def WBCgainsInWorld(self, yaw):
        # this function is equivalent to execute R.T @ K @ R, but faster
        w_R_hf = pin.rpy.rpyToMatrix(0, 0,yaw)

        self.kp_linW = w_R_hf.T @ self.kp_lin @ w_R_hf
        self.kd_linW = w_R_hf.T @ self.kd_lin @ w_R_hf
        self.kp_angW = w_R_hf.T @ self.kp_ang @ w_R_hf
        self.kd_angW = w_R_hf.T @ self.kd_ang @ w_R_hf


    def virtualImpedanceWrench(self, basePoseW, comPoseW, baseTwistW, comTwistW, des_pose, des_twist, des_acc = None, centroidalInertiaB = np.eye(6), comControlled = True):
        if not(des_pose is None or des_twist is None):
            if comControlled:
                act_pose = comPoseW
                act_twist = comTwistW
            else:
                act_pose = basePoseW
                act_twist = baseTwistW

            yaw = self.u.angPart(basePoseW)[2]
            self.WBCgainsInWorld(yaw)
            # FEEDBACK WRENCH
            # ---> linear part
            self.wrench_fbW[self.u.sp_crd["LX"]:self.u.sp_crd["LX"] + 3] = self.kp_linW @ (self.u.linPart(des_pose)  - self.u.linPart(act_pose)) + \
                                                                       self.kd_linW @ (self.u.linPart(des_twist) - self.u.linPart(act_twist))

            # ---> angular part
            # actual orientation: self.b_R_w
            # Desired Orientation

            # the following codes are equivalent but the second is faster
            # w_err = computeOrientationError(self.b_R_w.T, w_R_des)

            # faster
            start = time.time()
            w_R_des = pin.rpy.rpyToMatrix(self.u.angPart(des_pose))
            b_R_w = pin.rpy.rpyToMatrix(self.u.angPart(basePoseW)).T
            # compute orientation error
            b_R_des = b_R_w @ w_R_des
            # express orientation error in angle-axis form
            aa_err = pin.AngleAxis(b_R_des)
            b_err = aa_err.angle * aa_err.axis
            # the orientation error is expressed in the base_frame so it should be rotated to have the wrench in the
            # world frame
            w_err = b_R_w.T @ b_err

            # Note we defined the angular part of the des twist as omega
            self.wrench_fbW[self.u.sp_crd["AX"]:self.u.sp_crd["AX"] + 3] = self.kp_angW @ w_err + \
                                                                           self.kd_angW @ ( self.u.angPart(des_twist) - self.u.angPart(act_twist) )


            # FEED-FORWARD WRENCH
            if (des_acc is not None):
                # ---> linear part
                self.wrench_ffW[self.u.sp_crd["LX"]:self.u.sp_crd["LX"] + 3] = self.robot.robotMass * self.u.linPart(des_acc)
                # ---> angular part
                # compute inertia in the world frame:  w_I = R' * B_I * R
                w_I = b_R_w.T @ centroidalInertiaB @ b_R_w

                self.wrench_ffW[self.u.sp_crd["AX"]:self.u.sp_crd["AX"] + 3] = w_I @ self.u.angPart(des_acc)

        else:
            self.wrench_fbW[:] = 0
            self.wrench_ffW[:] = 0
        # GRAVITY WRENCH
        # ---> linear part
        # self.wrench_gW[self.u.sp_crd["LZ"]+1] = self.robot.robotMass * self.g_mag (to avoid unuseful repetition, this is in the definiton of wrench_gW)
        # ---> angular part
        if not comControlled:  # act_state  = base position in this case
            W_base_to_com = self.u.linPart(comPoseW) - self.u.linPart(basePoseW)
            self.wrench_gW[self.u.sp_crd["AX"]:self.u.sp_crd["AX"]+3] = np.cross(W_base_to_com, self.wrench_gW[self.u.sp_crd["LX"]:self.u.sp_crd["LX"]+3])
        # else the angular wrench is zero



    # Whole body controller that includes ffwd wrench + fb wrench (Virtual PD) + gravity compensation
    # all vector is in the wf
    def computeWBC(self, W_contacts, wJ, h_joints,  basePoseW= np.zeros(6), comPoseW= np.zeros(6), baseTwistW = np.zeros(6), comTwistW= np.zeros(6), des_pose = None, des_twist = None, des_acc = None,centroidalInertiaB = np.eye(6), comControlled = True,  type = 'projection', stance_legs=[True, True, True, True], numberOfJointsPerLeg = 3):
        # does side effect on tau_ffwd
        self.virtualImpedanceWrench(basePoseW, comPoseW, baseTwistW, comTwistW, des_pose, des_twist, des_acc, centroidalInertiaB, comControlled)
        if self.real_robot:
            self.wrench_desW = self.wrench_fbW + self.wrench_gW
            self.wrench_ffW[:] = 0
        else:
            self.wrench_desW = self.wrench_fbW + self.wrench_gW + self.wrench_ffW

        # wrench = NEMatrix @ grfs
        for leg in range(len(self.robot_params['ee_frames'])):
            start_col = 3 * leg
            end_col = 3 * (leg + 1)
            if stance_legs[leg]:#self.contact_state[leg]:
                # ---> linear part
                # identity matrix (I avoid to rewrite zeros)
                self.NEMatrix[self.u.sp_crd["LX"], start_col] = 1.
                self.NEMatrix[self.u.sp_crd["LY"], start_col + 1] = 1.
                self.NEMatrix[self.u.sp_crd["LZ"], start_col + 2] = 1.
                # ---> angular part
                # all in a function
                if comControlled:
                    self.NEMatrix[self.u.sp_crd["AX"]:self.u.sp_crd["AZ"] + 1, start_col:end_col] = \
                        pin.skew(W_contacts[leg] - self.u.linPart(comPoseW))
                else:
                    self.NEMatrix[self.u.sp_crd["AX"]:self.u.sp_crd["AZ"] + 1, start_col:end_col] = \
                        pin.skew(W_contacts[leg] - self.u.linPart(basePoseW))
            else:
                # clean the matrix (where there are zeros the grf will be zero and so the torques)
                self.NEMatrix[:, start_col:end_col] = 0.

        grForcesW_wbc = np.zeros(self.robot.na)

        # Map the desired wrench to grf
        if type == 'projection':
            grForcesW_wbc = self.projectionWBC()
        elif type == 'qp':
            grForcesW_wbc = self.qpWBC()

        tau_ffwd = np.zeros(self.robot.na)

        for leg in range(4):#(where there are zeros in the Matrix, the grf will be zero and so the torques, no need to check stance legs here)
            tau_leg = self.u.getLegJointState(leg, h_joints, numberOfJointsPerLeg=numberOfJointsPerLeg) - \
                      wJ[leg].T @ self.u.getLegJointState(leg, grForcesW_wbc, numberOfJointsPerLeg=numberOfJointsPerLeg)
            self.u.setLegJointState(leg, tau_leg, tau_ffwd, numberOfJointsPerLeg=numberOfJointsPerLeg)

        return tau_ffwd, grForcesW_wbc

    def projectionWBC(self, tol=1e-4):
        # NEMatrix is 6 x 12
        Npinv = np.linalg.pinv(self.NEMatrix.T, tol).T# self.NEMatrix.T @ np.linalg.inv(self.NEMatrix @ self.NEMatrix.T)
        return Npinv  @ self.wrench_desW

    def setWBCConstraints(self, normals = [np.array([0, 0, 1])]*4, friction_coeffs= [0.8]*4, reg = 1e-4):
        # this must be called at least once
        # uses inner piramid approx of friction cones
        # friction coeff must be the one in xacros. it is rescaled by the function
        C_leg = [None] * 4

        for leg in range(4):
            ty = np.cross(normals[leg], np.array([1, 0, 0]))
            tx = np.cross(ty, normals[leg])
            coeff_per_normal = friction_coeffs[leg]/np.sqrt(2.0) * normals[leg]
            C_leg[leg] = np.array([
                tx - coeff_per_normal,
                -tx - coeff_per_normal,
                ty - coeff_per_normal,
                -ty - coeff_per_normal])

        self.C_qp = block_diag(block_diag(C_leg[0], C_leg[1], C_leg[2], C_leg[3]))
        self.d_qp = np.zeros(self.C_qp.shape[0])
        self.reg_mat = np.eye(12) * 1e-4


    def qpWBC(self):
        #never profiled
        G = self.NEMatrix.T @ self.NEMatrix + self.reg_mat  # regularize and make it definite positive
        g = -self.NEMatrix.T @ self.wrench_desW

        w_des_grf = quadprog_solve_qp(G, g, self.C_qp, self.d_qp, None , None)
        return w_des_grf
