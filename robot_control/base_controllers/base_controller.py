
# -*- coding: utf-8 -*-
"""
Created on 3 May  2022

@author: mfocchi
"""

import threading
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty

#gazebo messages
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.msg import ModelState
#gazebo services
from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsPropertiesRequest

# ros utils
from gazebo_msgs.srv import ApplyBodyWrench

#other utils
from base_controllers.utils.ros_publish import RosPub
from base_controllers.utils.pidManager import PidManager
from base_controllers.utils.math_tools import *
from base_controllers.utils.common_functions import *
import pinocchio as pin
from ros_impedance_controller.msg import EffortPid

#dynamics
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
import  base_controllers.params as conf
robotName = "solo"

class BaseController(threading.Thread):
    """
        This Class can be used to simulate floating base robots that
        have an under-actuated base (e.g. like quadrupeds, mobile robots)

        ...

        Attributes
        ----------
        q, q_des : numpy array
           actual /desired joint positions
        qd, qd_des : numpy array
            actual /desired joint velocities
        tau, tau_ffwd :  numpy array
            actual /feed-forward torques
        basePoseW : numpy array
            base position (slice [0:3]) and orientation in Euler angles (slice [3:6])
        baseTwistW : numpy array
            base velocity linear (slice [0:3]) and angular (slice [3:6])
        b_R_w : numpy 2D array
            rotation matrix from base_link frame to world frame
        W_contacts : list of numpy arrays
            position of the feet expressed in the world frame
        grForcesW :  list of numpy arrays

        Methods
        -------
        loadModelAndPublishers(xacro_path=None)
           loads publishers for visual features (ros_pub), joint commands and declares subscriber to /ground_truth and /joint_states
        startSimulator(world_name = None)
           Starts gazebo simulator with ros_impedance_controller
        send_des_jstate(self, q_des, qd_des, tau_ffwd)
            publishes /command topic with set-points for joint positions, velocities and feed-forward torques
        startupProcedure():
            initialize PD gains
        initVars()
            initializes class variables
        logData()
            fill in the X_log variables for plotting purposes, it needs to be called at every loop
        _receive_jstate(msg)
            callback associated to the joint state subscriber, fills in q, qd, tau arrays
        _receive_pose(msg)
            callback associated to the ground truth subscriber, fills in base pose, orientation
            (quaternion, Euler angles), and base velocity (twist), and publishes the fixed transform between world and base_link

    """
    
    def __init__(self, robot_name="hyq", launch_file=None, external_conf = None, broadcast_world = True):
        threading.Thread.__init__(self)
        if (external_conf is not None):
            conf.robot_params = external_conf.robot_params
        self.robot_name = robot_name
        self.base_offset = np.array([conf.robot_params[self.robot_name]['spawn_x'],
                                     conf.robot_params[self.robot_name]['spawn_y'],
                                     conf.robot_params[self.robot_name]['spawn_z'],
                                     conf.robot_params[self.robot_name].get('spawn_R', 0.),
                                     conf.robot_params[self.robot_name].get('spawn_P', 0.),
                                     conf.robot_params[self.robot_name].get('spawn_Y', 0.)])

        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        self.u = Utils()
        self.math_utils = Math()
        # send data to param server
        self.verbose = conf.verbose
        self.custom_locosim_launch_file = False
        self.use_ground_truth_contacts = False
        self.apply_external_wrench = False
        self.time_external_wrench = 0.6

        self.use_torque_control = False
        self.real_robot = conf.robot_params[self.robot_name].get('real_robot', False)
        self.broadcast_world = broadcast_world
        self.publish_contact_gt_in_wf = False
        print("Initialized basecontroller---------------------------------------------------------------")

    def startSimulator(self, world_name = None, launch_file = None, additional_args = None):
        # clean up previous process
        os.system("killall rosmaster rviz gzserver gzclient")

        # gazebo finds the model plugins of the robot descriptions (e.g. joint armature) only through GAZEBO_PLUGIN_PATH
        plugin_dir = os.environ['LOCOSIM_DIR'] + '/robot_descriptions/gazebo_plugins/lib'
        plugin_paths = [p for p in os.environ.get('GAZEBO_PLUGIN_PATH', '').split(':') if p]
        if plugin_dir not in plugin_paths:
            os.environ['GAZEBO_PLUGIN_PATH'] = ':'.join([plugin_dir] + plugin_paths)

        if launch_file is None:
            if self.custom_locosim_launch_file:
                launch_file = rospkg.RosPack().get_path('ros_impedance_controller') + '/launch/ros_impedance_controller_' + self.robot_name + '.launch'
            else:
                launch_file = rospkg.RosPack().get_path('ros_impedance_controller') + '/launch/ros_impedance_controller_floating.launch'

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [launch_file,
                    'robot_name:=' + self.robot_name,
                    'spawn_x:=' + str(conf.robot_params[self.robot_name]['spawn_x']),
                    'spawn_y:=' + str(conf.robot_params[self.robot_name]['spawn_y']),
                    'spawn_z:=' + str(conf.robot_params[self.robot_name]['spawn_z']),
                    'spawn_R:=' + str(conf.robot_params[self.robot_name].get('spawn_R', 0.)),
                    'spawn_P:=' + str(conf.robot_params[self.robot_name].get('spawn_P', 0.)),
                    'spawn_Y:=' + str(conf.robot_params[self.robot_name].get('spawn_Y', 0.)),
                    'real_robot:=' + str(self.real_robot)]
        if world_name is not None:
            print(colored("Setting custom model: "+str(world_name), "blue"))
            cli_args.append('world_name:=' + str(world_name))
        if additional_args is not None:
            cli_args.extend(additional_args)
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        ros.sleep(1.0)
        print(colored('SIMULATION Started', 'blue'))


    def loadModelAndPublishers(self, xacro_path=None):
        # Loading a robot model of robot (Pinocchio)
        self.robot = getRobotModelFloating(self.robot_name)

        # instantiating objects
        self.ros_pub = RosPub(self.robot_name, only_visual=True)

        self.pub_des_jstate = ros.Publisher("/command", JointState, queue_size=1, tcp_nodelay=True)
        # freeze base  and pause simulation service
        self.reset_world = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_physics_client = ros.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.get_physics_client = ros.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        self.pause_physics_client = ros.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = ros.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.u.putIntoGlobalParamServer("verbose", self.verbose)

        self.apply_body_wrench = ros.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.broadcaster = SafeTFBroadcaster()

    def initSubscribers(self):
        self.sub_jstate = ros.Subscriber("/" + self.robot_name + "/joint_states", JointState,
                                         callback=self._receive_jstate, queue_size=1, tcp_nodelay=True)
        self.sub_pid_effort = ros.Subscriber("/" + self.robot_name + "/effort_pid", EffortPid,
                                             callback=self._receive_pid_effort, queue_size=1, tcp_nodelay=True)
        self.sub_pose = ros.Subscriber("/" + self.robot_name + "/ground_truth", Odometry, callback=self._receive_pose,
                                       queue_size=1, tcp_nodelay=True)

        if self.use_ground_truth_contacts:

            self.sub_contact_lf = ros.Subscriber("/" + self.robot_name + "/"+conf.robot_params[self.robot_name]['ee_frames'][0]+"_bumper", ContactsState,
                                                 callback=self._receive_contact_lf, queue_size=1, buff_size=2 ** 24,
                                                 tcp_nodelay=True)
            self.sub_contact_rf = ros.Subscriber("/" + self.robot_name + "/"+conf.robot_params[self.robot_name]['ee_frames'][1]+"_bumper", ContactsState,
                                                 callback=self._receive_contact_rf, queue_size=1, buff_size=2 ** 24,
                                                 tcp_nodelay=True)
            self.sub_contact_lh = ros.Subscriber("/" + self.robot_name +  "/"+conf.robot_params[self.robot_name]['ee_frames'][2]+"_bumper", ContactsState,
                                                 callback=self._receive_contact_lh, queue_size=1, buff_size=2 ** 24,
                                                 tcp_nodelay=True)
            self.sub_contact_rh = ros.Subscriber("/" + self.robot_name +  "/"+conf.robot_params[self.robot_name]['ee_frames'][3]+"_bumper", ContactsState,
                                                 callback=self._receive_contact_rh, queue_size=1, buff_size=2 ** 24,
                                                 tcp_nodelay=True)


        
        

    def _receive_contact_lf(self, msg):
        grf = np.zeros(3)
        if msg.states:
            grf[0] = msg.states[0].wrenches[0].force.x
            grf[1] =  msg.states[0].wrenches[0].force.y
            grf[2] =  msg.states[0].wrenches[0].force.z
        self.u.setLegJointState(self.u.leg_map["LF"], grf, self.grForcesLocal_gt)

    def _receive_contact_rf(self, msg):
        grf = np.zeros(3)
        if msg.states:
            grf[0] = msg.states[0].wrenches[0].force.x
            grf[1] =  msg.states[0].wrenches[0].force.y
            grf[2] =  msg.states[0].wrenches[0].force.z
        self.u.setLegJointState(self.u.leg_map["RF"], grf, self.grForcesLocal_gt)

    def _receive_contact_lh(self, msg):
        grf = np.zeros(3)
        if   msg.states:
            grf[0] = msg.states[0].wrenches[0].force.x
            grf[1] =  msg.states[0].wrenches[0].force.y
            grf[2] =  msg.states[0].wrenches[0].force.z
        self.u.setLegJointState(self.u.leg_map["LH"], grf, self.grForcesLocal_gt)

    def _receive_contact_rh(self, msg):
        grf = np.zeros(3)
        if  msg.states:
            grf[0] = msg.states[0].wrenches[0].force.x
            grf[1] =  msg.states[0].wrenches[0].force.y
            grf[2] =  msg.states[0].wrenches[0].force.z
        self.u.setLegJointState(self.u.leg_map["RH"], grf, self.grForcesLocal_gt)

    def _receive_pose(self, msg):
        self.quaternion[0] = msg.pose.pose.orientation.x
        self.quaternion[1] = msg.pose.pose.orientation.y
        self.quaternion[2] = msg.pose.pose.orientation.z
        self.quaternion[3] = msg.pose.pose.orientation.w

        self.basePoseW[self.u.sp_crd["LX"]] = msg.pose.pose.position.x
        self.basePoseW[self.u.sp_crd["LY"]] = msg.pose.pose.position.y
        self.basePoseW[self.u.sp_crd["LZ"]] = msg.pose.pose.position.z

        self.euler = np.array(euler_from_quaternion(self.quaternion))

        self.basePoseW[self.u.sp_crd["AX"]] = self.euler[0]
        self.basePoseW[self.u.sp_crd["AY"]] = self.euler[1]
        self.basePoseW[self.u.sp_crd["AZ"]] = self.euler[2]

        self.baseTwistW[self.u.sp_crd["LX"]] = msg.twist.twist.linear.x
        self.baseTwistW[self.u.sp_crd["LY"]] = msg.twist.twist.linear.y
        self.baseTwistW[self.u.sp_crd["LZ"]] = msg.twist.twist.linear.z
        self.baseTwistW[self.u.sp_crd["AX"]] = msg.twist.twist.angular.x
        self.baseTwistW[self.u.sp_crd["AY"]] = msg.twist.twist.angular.y
        self.baseTwistW[self.u.sp_crd["AZ"]] = msg.twist.twist.angular.z

        # compute orientation matrix
        self.b_R_w = self.math_utils.rpyToRot(self.euler)


    def _receive_jstate(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.q[joint_idx] = msg.position[msg_idx]
                    self.qd[joint_idx] = msg.velocity[msg_idx]
                    self.tau[joint_idx] = msg.effort[msg_idx]

    def _receive_pid_effort(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.tau_fb[joint_idx] = msg.effort_pid[msg_idx]

    def send_des_jstate(self, q_des, qd_des, tau_ffwd, soft_limits = 0.9, clip_commands = False):
         # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
         msg = JointState()
         msg.name = self.joint_names
         if clip_commands:
             msg.position = np.clip(q_des,self.robot.model.lowerPositionLimit[-self.robot.na:] * soft_limits ,self.robot.model.upperPositionLimit[-self.robot.na:] * soft_limits)
             msg.velocity = np.clip(qd_des, -self.robot.model.velocityLimit[-self.robot.na:] * soft_limits ,self.robot.model.velocityLimit[-self.robot.na:] * soft_limits)
             msg.effort = np.clip(tau_ffwd, -self.robot.model.effortLimit[-self.robot.na:] * soft_limits ,self.robot.model.effortLimit[-self.robot.na:] * soft_limits)
         else:
             msg.position = q_des
             msg.velocity = qd_des
             msg.effort = tau_ffwd
         self.pub_des_jstate.publish(msg)


    def deregister_node(self):
        print( "deregistering nodes"     )
        self.ros_pub.deregister_node()
        os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")    
        os.system(" rosnode kill /gazebo")
        os.system("pkill rosmaster")


    def get_contact(self):
        return self.W_contacts
    def get_pose(self):
        return self.basePoseW
    def get_jstate(self):
        return self.q
        
    def setGravity(self, value):
        # get actual configs
        physics_props = self.get_physics_client()         
       
        req_reset_gravity = SetPhysicsPropertiesRequest()
        #ode config
        req_reset_gravity.time_step = physics_props.time_step
        req_reset_gravity.max_update_rate = physics_props.max_update_rate           
        req_reset_gravity.ode_config =physics_props.ode_config
        req_reset_gravity.gravity =  physics_props.gravity
        req_reset_gravity.gravity.z =  value
        self.set_physics_client(req_reset_gravity)

    def freezeBase(self, flag, basePoseW=None, baseTwistW=None):
        print(colored("Freezing base", "blue"))
        if flag:
            self.setGravity(0)
        else:
            self.setGravity(-9.81)
        # create the message
        req_reset_world = SetModelStateRequest()
        # create model state
        model_state = ModelState()
        model_state.model_name = self.robot_name
        if basePoseW is None:
            model_state.pose.position.x = self.u.linPart(self.basePoseW)[0]
            model_state.pose.position.y = self.u.linPart(self.basePoseW)[1]
            model_state.pose.position.z = self.u.linPart(self.basePoseW)[2]

            model_state.pose.orientation.x = self.quaternion[0]
            model_state.pose.orientation.y = self.quaternion[1]
            model_state.pose.orientation.z = self.quaternion[2]
            model_state.pose.orientation.w = self.quaternion[3]
        else:
            model_state.pose.position.x = self.u.linPart(basePoseW)[0]
            model_state.pose.position.y = self.u.linPart(basePoseW)[1]
            model_state.pose.position.z = self.u.linPart(basePoseW)[2]

            quaternion = pin.Quaternion(pin.rpy.rpyToMatrix(self.u.angPart(basePoseW)))
            model_state.pose.orientation.x = quaternion.x
            model_state.pose.orientation.y = quaternion.y
            model_state.pose.orientation.z = quaternion.z
            model_state.pose.orientation.w = quaternion.w

        if baseTwistW is None:
            model_state.twist.linear.x = self.u.linPart(self.baseTwistW)[0]
            model_state.twist.linear.y = self.u.linPart(self.baseTwistW)[1]
            model_state.twist.linear.z = self.u.linPart(self.baseTwistW)[2]

            model_state.twist.angular.x = self.u.angPart(self.baseTwistW)[0]
            model_state.twist.angular.y = self.u.angPart(self.baseTwistW)[1]
            model_state.twist.angular.z = self.u.angPart(self.baseTwistW)[2]
        else:
            model_state.twist.linear.x = self.u.linPart(baseTwistW)[0]
            model_state.twist.linear.y = self.u.linPart(baseTwistW)[1]
            model_state.twist.linear.z = self.u.linPart(baseTwistW)[2]

            model_state.twist.angular.x = self.u.angPart(baseTwistW)[0]
            model_state.twist.angular.y = self.u.angPart(baseTwistW)[1]
            model_state.twist.angular.z = self.u.angPart(baseTwistW)[2]

        req_reset_world.model_state = model_state
        # send request and get response (in this case none)
        self.reset_world(req_reset_world)

    def mapBaseToWorld(self, B_var):
        W_var = self.b_R_w.transpose().dot(B_var) + self.u.linPart(self.basePoseW)                            
        return W_var
                                                                                                                                
    def updateKinematics(self):
        # q is continuously updated
        # to compute in the base frame  you should put neutral base
        # b_X_w = motionVectorTransform(np.zeros(3), self.b_R_w)
        # b_X_w = pin.SE3(self.b_R_w, np.zeros(3)).action
        # self.gen_velocities[:6] = b_X_w.dot(self.baseTwistW)

        #the v (generalized velocities) must follow Pinocchio's internal convention,
        #which includes the base velocity expressed in the local (body) frame, i.e., the frame of the floating base.
        self.gen_velocities[:3] = self.b_R_w.dot(self.u.linPart(self.baseTwistW))
        self.gen_velocities[3:6] = self.b_R_w.dot(self.u.angPart(self.baseTwistW))
        self.gen_velocities[6:] = self.qd
        self.neutral_fb_jointstate[7:] = self.q
        pin.forwardKinematics(self.robot.model, self.robot.data, self.neutral_fb_jointstate, self.gen_velocities)
        pin.computeJointJacobians(self.robot.model, self.robot.data)  
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        ee_frames = conf.robot_params[self.robot_name]['ee_frames']
        for leg in range(4):
            self.B_contacts[leg] = self.robot.framePlacement(self.neutral_fb_jointstate,
                                                             self.robot.model.getFrameId(ee_frames[leg]),
                                                             update_kinematics=False ).translation.copy()
            self.W_contacts[leg] = self.mapBaseToWorld(self.B_contacts[leg].transpose()) # as an alternative I could have used self.configuration in place of self.neutral_fb_jointstate in framePlacement
        if self.use_ground_truth_contacts:
            for leg in range(4):
                self.w_R_lowerleg[leg] = self.b_R_w.transpose().dot(self.robot.data.oMf[self.lowerleg_index[leg]].rotation)

        for leg in range(4):
            self.J[leg] = self.robot.frameJacobian(self.neutral_fb_jointstate,
                                                   self.robot.model.getFrameId(ee_frames[leg]),
                                                   update=False,
                                                   ref_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,6+leg*self.numberOfJointsPerLeg:6+(leg+1)*self.numberOfJointsPerLeg]
            self.wJ[leg] = self.b_R_w.transpose().dot(self.J[leg])
            try:
                self.J_inv[leg] = np.linalg.inv(self.J[leg])
                self.wJ_inv[leg] = self.J_inv[leg].dot(self.b_R_w)
            except np.linalg.linalg.LinAlgError as error:
                self.J_inv[leg][:,:] = 0.
                self.wJ_inv[leg][:,:] = 0.

        # Pinocchio Update the joint and frame placements
        self.configuration[:3] = self.u.linPart(self.basePoseW)
        self.configuration[3:7] = self.quaternion
        self.configuration[7:] = self.q

        self.M = self.robot.mass(self.configuration)
        self.h = pin.nonLinearEffects(self.robot.model, self.robot.data, self.configuration, self.gen_velocities)
        self.h_joints = self.h[6:]
        self.g = self.robot.gravity(self.configuration)
        self.g_joints = self.g[6:]
        
        #compute contact forces
        self.estimateContactForces()
        #self.estimateContactForcesFloating()  # Robot Dynamics Lecture Notes, eq. 3.61
        self.estimateLegContacts()

        # compute com / robot inertias
        self.comPosB, self.comVelB = copy.deepcopy(self.robot.robotComB(self.q, self.qd))
        self.comPoseW = copy.deepcopy(self.basePoseW)
        self.comPoseW[self.u.sp_crd["LX"]:self.u.sp_crd["LX"]+3] = self.robot.robotComW(self.configuration) # + np.array([0.05, 0.0,0.0])
        W_base_to_com = self.u.linPart(self.comPoseW)  - self.u.linPart(self.basePoseW)
        # self.comTwistW = np.dot( motionVectorTransform( W_base_to_com, np.eye(3)),self.baseTwistW)
        self.comTwistW = pin.SE3(np.eye(3), W_base_to_com).action.dot(self.baseTwistW)

        # inertia w.r.t the com
        self.centroidalInertiaB = self.robot.centroidalInertiaB(self.configuration, self.gen_velocities)
        # inertia w.r.t the base frame origin
        self.compositeRobotInertiaB = self.robot.compositeRobotInertiaB(self.configuration)


        if self.broadcast_world:
            self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
                                       self.quaternion,
                                       ros.Time.now(), '/base_link', '/world')

    def estimateContactForces(self):           
        # estimate ground reaction forces from tau
        for leg in range(4):
            grf = self.wJ_inv[leg].T.dot(self.u.getLegJointState(leg,  self.h_joints-self.tau ))
            self.u.setLegJointState(leg, grf, self.grForcesW)

    def estimateContactForcesFloating(self):
        # #computes contact forces from constraints F = (J*M^-1JT)^-1*(J*M^-1*(S^T*tau -h) +Jd*qd)
        Jc = np.empty((0,self.robot.nv))
        Jdqd = np.empty((0))
        for leg in range(4):
            frame_id = self.robot.model.getFrameId(conf.robot_params[self.robot_name]['ee_frames'][leg])
            # I use self.neutral_fb_jointstate otherwise with self.configuration I get wrong results, and rotate for each leg
            Jdqd =  np.append(Jdqd, self.b_R_w.T @  self.robot.frameClassicAcceleration(self.neutral_fb_jointstate, self.gen_velocities, None, frame_id).linear)
            # to avoid computing the spatial Jacobian I first compute it in the Base frame (LOCAL_WORLD_ALIGNED) and then I rotate it
            Jleg = self.b_R_w.T @ self.robot.frameJacobian(self.neutral_fb_jointstate, frame_id, update=False, ref_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]
            Jc = np.vstack((Jc, Jleg))
        Lambda = np.linalg.inv(Jc @ (np.linalg.inv(self.M) @ Jc.T))
        S = np.hstack((np.zeros((self.robot.na, 6)), np.eye(self.robot.na)))
        self.grForcesW = -Lambda @ ( Jc @ np.linalg.inv(self.M) @ ( S.T @ self.tau -self.h) + Jdqd )

    def estimateLegContacts(self):
        if self.use_ground_truth_contacts:
            for leg in range(4):
                grfLocal_gt = self.u.getLegJointState(leg, self.grForcesLocal_gt)
                if self.publish_contact_gt_in_wf:  # if you spawn a robot platform it starts ti publish in WF
                    grf_gt = grfLocal_gt
                else:
                    grf_gt = self.w_R_lowerleg[leg] @ grfLocal_gt

                self.u.setLegJointState(leg, grf_gt, self.grForcesW_gt)
                # contact state is computed using gt forces if use_ground_truth_contacts == True (previous computation is overridden)
                if self.contact_normal[leg].dot(grf_gt) >= conf.robot_params[self.robot_name]['force_th']:
                    self.contact_state[leg] = True

                else:
                    self.contact_state[leg] = False
        else:
            for leg in range(4):
                grf = self.u.getLegJointState(leg, self.grForcesW)
                if self.contact_normal[leg].dot(grf) >= conf.robot_params[self.robot_name]['force_th']:
                    self.contact_state[leg] = True
                else:
                    self.contact_state[leg] = False
                    # if self.time % 100:
                    #     print('contact lost on leg: ' + conf.robot_params[self.robot_name]['ee_frames'][leg])

    def applyForce(self, Fx, Fy, Fz, Mx, My, Mz, duration, link_name="base_link"):
        from geometry_msgs.msg import Wrench, Point

        wrench = Wrench()
        wrench.force.x = Fx
        wrench.force.y = Fy
        wrench.force.z = Fz
        wrench.torque.x = Mx
        wrench.torque.y = My
        wrench.torque.z = Mz
        reference_frame = "world"  # you can apply forces only in this frame because this service is buggy, it will ignore any other frame
        reference_point = Point(x=0, y=0, z=0)
        try:
            self.apply_body_wrench(body_name=self.robot_name+"::"+link_name, reference_frame=reference_frame,
                                   reference_point=reference_point, wrench=wrench, duration=ros.Duration(duration))
        except:
            pass
                                 
    def startupProcedure(self):

            self.pid = PidManager(self.joint_names) #I start after cause it needs joint names filled in by receive jstate callback
            # set joint pdi gains
            self.pid.setPDjoints(conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'], np.zeros(self.robot.na))

            if (self.robot_name == 'hyq'):                        
                # these torques are to compensate the leg gravity
                self.gravity_comp = np.array(
                    [24.2571, 1.92, 50.5,  21.3801, -2.08377, -44.9598, 24.2, 1.92, 50.5739, 21.3858, -2.08365, -44.9615])
                                                        
                print("reset posture...")
                self.freezeBase(1, basePoseW=self.base_offset )
                start_t = ros.get_time()
                while ros.get_time() - start_t < 1.0:
                    self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
                    ros.sleep(0.01)
                if self.verbose:
                    print("q err prima freeze base", (self.q - self.q_des))
  
          
                print("put on ground and start compensating gravity...")
                self.freezeBase(0)                                                
                ros.sleep(0.5)
                if self.verbose:
                    print("q err pre grav comp", (self.q - self.q_des))
                                                        
                start_t = ros.get_time()
                while ros.get_time()- start_t < 1.0:
                    self.send_des_jstate(self.q_des, self.qd_des, self.gravity_comp)
                    ros.sleep(0.01)
                if self.verbose:
                    print("q err post grav comp", (self.q - self.q_des))
                                                        
                print("starting com controller (no joint PD)...")                
                self.pid.setPDs(0.0, 0.0, 0.0)
                self.use_torque_control = True
            
            if (self.robot_name == 'aliengo' or self.robot_name=='go1'):
                start_t = ros.get_time()
                while ros.get_time() - start_t < 0.5:
                    self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
                    ros.sleep(0.01)
                #self.pid.setPDs(0.0, 0.0, 0.0)

            if (self.robot_name == 'solo'):
                start_t = ros.get_time()
                while ros.get_time() - start_t < 0.5:
                    self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
                    ros.sleep(0.01)
            print(colored("finished startup -- starting controller", "red"))

    def initVars(self):
        self.basePoseW = np.zeros(6)
        self.baseTwistW = np.zeros(6)
        self.comPoseW = np.zeros(6)
        self.comTwistsW = np.zeros(6)
        self.stance_legs = np.array([True, True, True, True])
        self.centroidalInertiaB = np.eye(3)
        self.compositeRobotInertiaB = np.eye(3)
        self.gen_velocities = np.zeros(self.robot.nv)
        self.neutral_fb_jointstate = pin.neutral(self.robot.model)
        self.configuration = pin.neutral(self.robot.model)
        self.q = np.zeros(self.robot.na)
        self.qd = np.zeros(self.robot.na)
        self.tau = np.zeros(self.robot.na)
        self.tau_fb = np.zeros(self.robot.na)
        self.q_des = np.zeros(self.robot.na)
        self.quaternion = np.array([0., 0., 0., 1.]) #fundamental otherwise receivepose gets stuck
        self.euler_old = np.zeros(3)
        self.q_des = conf.robot_params[self.robot_name]['q_0']
        self.qd_des = np.zeros(self.robot.na)
        self.tau_ffwd = np.zeros(self.robot.na)
        self.gravity_comp = np.zeros(self.robot.na)
        self.b_R_w = np.eye(3)
        self.grForcesW = np.zeros(self.robot.na)
        self.grForcesLocal_gt = np.zeros(self.robot.na)
        self.grForcesW_gt = np.zeros(self.robot.na)

        self.J = self.u.listOfArrays(4, np.zeros((3,3)))
        self.wJ = self.u.listOfArrays(4, np.zeros((3,3)))
        self.J_inv = self.u.listOfArrays(4, np.zeros((3,3)))
        self.wJ_inv = self.u.listOfArrays(4, np.zeros((3,3)))
        self.W_contacts = self.u.listOfArrays(4, np.zeros((3)))
        self.W_contacts_des = self.u.full_listOfArrays(4, 3)
        self.B_contacts = self.u.listOfArrays(4, np.zeros((3)))
        self.B_contacts_des = self.u.full_listOfArrays(4, 3)
        self.contact_state = self.u.full_listOfArrays(4, 1, 0, False)
        self.contact_normal = self.u.listOfArrays(4, np.array([0., 0., 1]))
        self.w_R_lowerleg =  self.u.listOfArrays(4, np.eye(3))

        #log vars
        self.basePoseW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.baseTwistW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.comPoseW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.comTwistW_log = np.full((6, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.q_des_log = np.full((self.robot.na, conf.robot_params[self.robot_name]['buffer_size']), np.nan)    
        self.q_log = np.full((self.robot.na, conf.robot_params[self.robot_name]['buffer_size']), np.nan)   
        self.qd_des_log = np.full((self.robot.na, conf.robot_params[self.robot_name]['buffer_size']), np.nan)    
        self.qd_log = np.full((self.robot.na, conf.robot_params[self.robot_name]['buffer_size']), np.nan)                                  
        self.tau_ffwd_log = np.full((self.robot.na, conf.robot_params[self.robot_name]['buffer_size']), np.nan)    
        self.tau_log = np.full((self.robot.na, conf.robot_params[self.robot_name]['buffer_size']), np.nan)                                  
        self.grForcesW_log = np.full((self.robot.na, conf.robot_params[self.robot_name]['buffer_size']), np.nan) 
        self.time_log = np.full((conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        self.constr_viol_log = np.full((4, conf.robot_params[self.robot_name]['buffer_size']), np.nan)
        
        self.time = np.zeros(1)
        self.loop_time = conf.robot_params[self.robot_name]['dt']
        self.log_counter = 0


        # order: lf lh rf rh
        if self.use_ground_truth_contacts:
            self.lowerleg_index = [0]*4
            self.lowerleg_frame_names = []
            for f in self.robot.model.frames:
                if 'lower' in f.name:
                    self.lowerleg_frame_names.append(f.name)
            self.lowerleg_frame_names = self.u.mapLegListToRos(self.lowerleg_frame_names)

            for legid in self.u.leg_map.keys():
                leg = self.u.leg_map[legid]
                self.lowerleg_index[leg] =  self.robot.model.getFrameId(self.lowerleg_frame_names[leg])

        self.numberOfJointsPerLeg = self.getNumberOfJointsPerLeg()

    def getNumberOfJointsPerLeg(self):
        ee_frames = conf.robot_params[self.robot_name]['ee_frames']
        first_foot_frame_id = self.robot.model.getFrameId(ee_frames[0])
        joint_id = self.robot.model.frames[first_foot_frame_id].parentJoint
        # Walk up the kinematic tree starting from the parent joint
        count = 0
        while joint_id != 1:
            count += 1
            joint_id = self.robot.model.parents[joint_id]
        return count



    def logData(self):
        if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
            self.basePoseW_log[:, self.log_counter] = self.basePoseW
            self.baseTwistW_log[:, self.log_counter] =  self.baseTwistW
            self.q_des_log[:, self.log_counter] =  self.q_des
            self.q_log[:,self.log_counter] =  self.q  
            self.qd_des_log[:,self.log_counter] =  self.qd_des
            self.qd_log[:,self.log_counter] = self.qd                       
            self.tau_ffwd_log[:,self.log_counter] = self.tau_ffwd                    
            self.tau_log[:,self.log_counter] = self.tau                     
            self.grForcesW_log[:,self.log_counter] =  self.grForcesW
            self.time_log[self.log_counter] = self.time
            self.log_counter+=1

    def sync_check(self):
        new_time = ros.Time.now().to_sec()
        if  hasattr(self, 'ros_time'):
            self.loop_time = new_time-self.ros_time
            # if  self.loop_time> conf.robot_params[self.robot_name]['dt']*1.05:
            #     print('Out of loop frequency')
        self.ros_time = new_time

    def setSimSpeed(self, dt_sim=0.001, max_update_rate=1000, iters=50):
        physics_req = SetPhysicsPropertiesRequest()
        physics_req.time_step = dt_sim
        physics_req.max_update_rate = max_update_rate
        physics_req.ode_config.sor_pgs_iters = iters
        physics_req.ode_config.sor_pgs_w = 1.3
        physics_req.ode_config.contact_surface_layer = 0.001
        physics_req.ode_config.contact_max_correcting_vel = 100
        physics_req.ode_config.erp = 0.2
        physics_req.ode_config.max_contacts = 20
        physics_req.gravity.z = -9.81
        self.set_physics_client(physics_req)
	
def talker(p):
    p.start()
    if  (p.robot_name == 'solo_fw'):
        p.custom_locosim_launch_file = True
    p.startSimulator()
    p.loadModelAndPublishers()
    p.initVars()
    p.initSubscribers()
    p.startupProcedure()

    #loop frequency       
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    # use this if rostopic hz /command it gives frequency < rate
    #p.setSimSpeed(max_update_rate=100)
    #control loop
    while not ros.is_shutdown():
        #update the kinematics
        p.updateKinematics()

        # implement your own controller
        if p.use_torque_control:
            p.pid.setPDs(0,0,0)
            p.tau_ffwd = conf.robot_params[p.robot_name]['kp'] * np.subtract(p.q_des,   p.q)  - conf.robot_params[p.robot_name]['kd']*p.qd + p.gravity_comp
            #p.tau_ffwd[12:] =0.01 * np.subtract(p.q_des[12:],   p.q[12:])  - 0.001*p.qd[12:]
        else:
            p.tau_ffwd = np.zeros(p.robot.na)

        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd, clip_commands=p.real_robot)
          
	    # log variables
        p.logData()    
        
        # plot actual (green) and desired (blue) contact forces 
        for leg in range(4):
            p.ros_pub.add_arrow(p.W_contacts[leg], p.contact_state[leg]*p.u.getLegJointState(leg, p.grForcesW/(6*p.robot.robotMass)),"green")
            if (p.use_ground_truth_contacts):
                p.ros_pub.add_arrow(p.W_contacts[leg], p.u.getLegJointState(leg, p.grForcesW_gt / (6 * p.robot.robotMass)), "red")
        p.ros_pub.publishVisual()

        #wait for synconization of the control loop
        rate.sleep()     
        #p.sync_check()
        p.time = np.round(p.time + np.array([p.loop_time]), 4) # to avoid issues of dt 0.0009999

        if (p.apply_external_wrench and p.time > p.time_external_wrench):
            print("START APPLYING EXTERNAL WRENCH")
            p.applyForce(0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.05)
            p.apply_external_wrench = False


if __name__ == '__main__':
    p = BaseController(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        if conf.plotting:
            plotJoint('position', time_log=p.time_log, q_log=p.q_log, q_des_log=p.q_des_log, joint_names=p.joint_names)
            plotFrame('position', time_log=p.time_log, Pose_log=p.basePoseW_log,
                      title='CoM', frame='W', sharex=True, sharey=False, start=0, end=-1)
            plotJoint('torque', time_log=p.time_log, tau_log=p.tau_log, joint_names=p.joint_names)




