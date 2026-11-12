# Description
# File contains some necessary control algorithms for HyQ
# Author: Michele Focchi
# Date: 04-12-2022
import rospy as ros
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import numpy as np

class TrajectoryManager():
    def __init__(self, robot_name, conf, gripper_manager):
        self.robot_name = robot_name
        self.conf = conf
        self.gripper_manager = gripper_manager
        self.joint_names = conf['joint_names']
        self.real_robot = conf['real_robot']
        if self.real_robot:
            self.traj_controller = "scaled_pos_joint_traj_controller"
        else:
            self.traj_controller = "pos_joint_traj_controller"

    # basic one, no gripper joints
    def send_joint_trajectory(self):
        # Creates a trajectory and sends it using the selected action server
        trajectory_client = actionlib.SimpleActionClient("{}/follow_joint_trajectory".format(
            "/" + self.robot_name + "/" + self.traj_controller), FollowJointTrajectoryAction)
        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        # The following list are arbitrary positions
        # Change to your own needs if desired q0 [ 0.5, -0.7, 1.0, -1.57, -1.57, 0.5]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
        # position_list = [[0.5, -0.7, 1.0, -1.57, -1.57, 0.5]]  # limits([0,-pi], [-pi/2,pi/2],  [0, -pi])
        # position_list.append([0.5, -0.7 - 0.2, 1.0 - 0.1, -1.57, -1.57, 0.5])
        # position_list.append([0.5 + 0.5, -0.7 - 0.3, 1.0 - 0.1, -1.57, -1.57, 0.5])
        # position_list.append([0.5 + 0.5, -0.7 - 0.3, 1.0 , -1., -1.57, 0.5])

        self.q0 = self.conf['q_0']
        dq1 = np.array([0.2, 0, 0, 0, 0, 0])
        dq2 = np.array([0.2, -0.2, 0, 0, 0, 0])
        dq3 = np.array([0.2, -0.2, 0.4, 0, 0, 0])
        position_list = [self.q0]  # limits([0,-pi], [-pi/2,pi/2],  [0, -pi])
        position_list.append(self.q0 + dq1)
        position_list.append(self.q0 + dq2)
        position_list.append(self.q0 + dq3)
        print(colored("List of targets for joints: ", 'blue'))
        print(position_list[0])
        print(position_list[1])
        print(position_list[2])
        print(position_list[3])

        duration_list = [5.0, 10.0, 20.0, 30.0]
        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = ros.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        self.ask_confirmation(position_list)
        print("Executing trajectory using the {}".format("pos_joint_traj_controller"))
        trajectory_client.send_goal(goal)

        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        print("Trajectory execution finished in state {}".format(result.error_code))

    # manages gripper joints
    def send_joint_trajectory_2(self):
        # Creates a trajectory and sends it using the selected action server
        trajectory_client = actionlib.SimpleActionClient("{}/follow_joint_trajectory".format(
            "/" + self.robot_name + "/" + self.traj_controller), FollowJointTrajectoryAction)
        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()

        # The following list are arbitrary positions
        # Change to your own needs if desired q0 [ 0.5, -0.7, 1.0, -1.57, -1.57, 0.5]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
        #print(colored("JOINTS ARE: ", 'blue'), self.q.transpose())

        if self.real_robot:
            goal.trajectory.joint_names = self.joint_names
            position_list = [
                [-0.4253643194781702, -0.9192648094943543, -2.162015914916992, -1.621634145776266, -1.5201204458819788,
                 -2.2737816015826624]]  # go on 1 brick
            position_list.append(
                [-0.42451507249941045, -0.9235735100558777, -1.975731611251831, -1.8549186191954554, -1.534570042287008,
                 -2.1804688612567347])  # approach 1 brick and grasp
            position_list.append(
                [-0.2545421759234827, -1.2628285449794312, -2.049499988555908, -1.3982257705977936, -1.4819391409503382,
                 -2.4832173029529017])  # move 2 second brick
            position_list.append(
                [-0.2545355002032679, -1.2625364822200318, -1.910099983215332, -1.5169030030122777, -1.4750459829913538,
                 -2.4462133089648646])  # approach 2 brick and release
            position_list.append(
                [-0.2544291655169886, -1.277967320089676, -2.1508238315582275, -1.2845929724029084, -1.465815846120016,
                 -2.445918385182516])  # evade and open gripper

            # print(colored("List of targets for joints: ",'blue'))
            # print(position_list[0])
            # print(position_list[1])
            # print(position_list[2])
            # print(position_list[3])
            # print(position_list[4])

            duration_list = [5.0, 5.0, 5.0, 5.0, 5]
            gripper_state = ['open', 'close', 'idle', 'open', 'open']
            gripper_diameter = [130, 45, 45, 65, 130]
            self.ask_confirmation(position_list)

            # set a different goal with gripper closed or opened
            for i, position in enumerate(position_list):
                point = JointTrajectoryPoint()
                point.positions = position
                point.time_from_start = ros.Duration(duration_list[i])
                # add a single goal and execute it
                goal.trajectory.points = [point]

                print("reaching position: ", position)
                trajectory_client.send_goal(goal)
                trajectory_client.wait_for_result()
                # while not trajectory_client.get_state() == 3:
                #     ros.sleep(1)
                #     print("reaching position: ", position)
                result = trajectory_client.get_result()
                print("Target Reached error code {}".format(result.error_code))
                if (gripper_state[i] == 'close'):
                    print("closing gripper")
                    p.controller_manager.gm.move_gripper(gripper_diameter[i])
                if (gripper_state[i] == 'open'):
                    print("opening gripper")
                    p.controller_manager.gm.move_gripper(gripper_diameter[i])
                time.sleep(2.)
        else:  # simulation need to manage gripper as desjoints
            finger_names = ['hand_1_joint', 'hand_2_joint', 'hand_3_joint']
            # select only the active fingers
            goal.trajectory.joint_names = self.joint_names + finger_names[:self.gripper_manager.number_of_fingers]
            q0 = self.conf['q_0'].tolist()
            q0.extend((np.ones(
                self.gripper_manager.number_of_fingers) * self.gripper_manager.mapToGripperJoints(
                40)).tolist())
            q1 = [-0.4253643194781702, -0.9192648094943543, -2.162015914916992, -1.621634145776266, -1.5201204458819788,
                  -2.2737816015826624]
            q1.extend((np.ones(
                self.gripper_manager.number_of_fingers) * self.gripper_manager.mapToGripperJoints(
                90)).tolist())
            q2 = [-0.42451507249941045, -0.9235735100558777, -1.975731611251831, -1.8549186191954554,
                  -1.534570042287008, -2.1804688612567347]
            q2.extend((np.ones(
                self.gripper_manager.number_of_fingers) * self.gripper_manager.mapToGripperJoints(
                90)).tolist())
            q3 = [-0.42451507249941045, -0.9235735100558777, -1.975731611251831, -1.8549186191954554,
                  -1.534570042287008, -2.1804688612567347]
            q3.extend((np.ones(
                self.gripper_manager.number_of_fingers) * self.gripper_manager.mapToGripperJoints(
                65)).tolist())
            q4 = [-0.2545421759234827, -1.2628285449794312, -2.049499988555908, -1.3982257705977936,
                  -1.4819391409503382, -2.4832173029529017]
            q4.extend((np.ones(
                self.gripper_manager.number_of_fingers) * self.gripper_manager.mapToGripperJoints(
                65)).tolist())
            q5 = [-0.2545355002032679, -1.2625364822200318, -1.910099983215332, -1.5169030030122777,
                  -1.4750459829913538, -2.4462133089648646]
            q5.extend((np.ones(
                self.gripper_manager.number_of_fingers) * self.gripper_manager.mapToGripperJoints(
                65)).tolist())
            q6 = [-0.2545355002032679, -1.2625364822200318, -1.910099983215332, -1.5169030030122777,
                  -1.4750459829913538, -2.4462133089648646]
            q6.extend((np.ones(
                self.gripper_manager.number_of_fingers) * self.gripper_manager.mapToGripperJoints(
                90)).tolist())
            q7 = [-0.2544291655169886, -1.277967320089676, -2.1508238315582275, -1.2845929724029084, -1.465815846120016,
                  -2.445918385182516]
            q7.extend((np.ones(
                self.gripper_manager.number_of_fingers) * self.gripper_manager.mapToGripperJoints(
                90)).tolist())
            q8 = [-0.2544291655169886, -1.277967320089676, -2.1508238315582275, -1.2845929724029084, -1.465815846120016,
                  -2.445918385182516]
            q8.extend((np.ones(
                self.gripper_manager.number_of_fingers) * self.gripper_manager.mapToGripperJoints(
                130)).tolist())

            position_list = [q0]  # go home
            position_list.append(q1)  # go on 1 brick
            position_list.append(q2)  # approach 1 brick
            position_list.append(q3)  # close gripper
            position_list.append(q4)  # move 2 second brick
            position_list.append(q5)  # approach 2 brick
            position_list.append(q6)  # open gripper
            position_list.append(q7)  # evade
            position_list.append(q8)  # open gripper

            duration_list = [5., 5.0, 5.0, 2., 5.0, 5.0, 2., 5., 2.]
            self.ask_confirmation(position_list)

            # set a different goal with gripper closed or opened
            for i, position in enumerate(position_list):
                point = JointTrajectoryPoint()
                point.positions = position
                point.time_from_start = ros.Duration(duration_list[i])
                # add a single goal and execute it
                goal.trajectory.points = [point]
                print("reaching position: ", position)
                trajectory_client.send_goal(goal)
                trajectory_client.wait_for_result()
                result = trajectory_client.get_result()
                print("Target Reached error code {}".format(result.error_code))

    def ask_confirmation(self, waypoint_list):
        """Ask the user for confirmation. This function is obviously not necessary, but makes sense
        in a testing script when you know nothing about the user's setup."""
        ros.logwarn("The robot will move to the following waypoints: \n{}".format(waypoint_list))
        confirmed = False
        valid = False
        while not valid:
            input_str = input(
                "Please confirm that the robot path is clear of obstacles.\n"
                "Keep the EM-Stop available at all times. You are executing\n"
                "the motion at your own risk. Please type 'y' to proceed or 'n' to abort: ")
            valid = input_str in ["y", "n"]
            if not valid:
                ros.loginfo("Please confirm by entering 'y' or abort by entering 'n'")
            else:
                if (input_str == "y"):
                    confirmed = True
        if not confirmed:
            ros.loginfo("Exiting as requested by user.")
            sys.exit(0)

