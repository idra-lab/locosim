# Description
# File contains some necessary control algorithms for HyQ
# Author: Michele Focchi
# Date: 23-10-2022
import math

import rospkg
import socket
import numpy as np
from base_controllers.components.filter import SecondOrderFilter
import rospy as ros
from std_srvs.srv import Trigger, TriggerRequest
from  termcolor import  colored
from ros_impedance_controller.srv import generic_float

class GripperManager():
    def __init__(self, gripper_type, real_robot_flag = False, dt = 0.001, gripping_duration = 5.):
        self.gripping_duration = gripping_duration
        self.real_robot = real_robot_flag
        self.gripper_type = gripper_type
        if self.gripper_type == 'soft_2':
            print(colored("Using soft 2 finger gripper!", "blue"))
            self.q_des_gripper = np.array([0., 0.])
            self.number_of_fingers = 2
        elif self.gripper_type == 'robotiq_2':
            print(colored("Using robotiq gripper!", "blue"))
            self.q_des_gripper = np.array([0.])
            self.number_of_fingers = 1
        else:
            print(colored("Using hard 3 finger gripper!", "blue"))
            self.q_des_gripper = np.array([1.8, 1.8, 1.8])
            self.number_of_fingers = 3

        self.SO_filter = SecondOrderFilter(self.number_of_fingers)
        self.SO_filter.initFilter(self.q_des_gripper,dt)
        ros.Service('move_gripper', generic_float, self.move_gripper_callback)

    def move_gripper_callback(self, req):
        diameter = req.data
        self.move_gripper(diameter)
        return True

    def resend_robot_program(self):
        ros.sleep(1.5)
        ros.wait_for_service("/ur5/ur_hardware_interface/resend_robot_program")
        sos_service = ros.ServiceProxy('/ur5/ur_hardware_interface/resend_robot_program', Trigger)
        sos = TriggerRequest()
        result = sos_service(sos)
        # print(result)
        ros.sleep(0.1)

    def mapToGripperJoints(self, diameter):
        if self.gripper_type == 'soft_2':
            D0 = 40
            L = 60
            delta =0.5*(diameter - D0)
            return math.atan2(delta, L)
        elif self.gripper_type == 'robotiq_2':
            return 0.85 -(diameter) * 0.85/85.  # D = 0.1  => 0, D = 0 => 0.8
        else:
            return (diameter - 22) / (130 - 22) * (-np.pi) + np.pi  # D = 130-> q = 0, D = 22 -> q = 3.14

    def getDesGripperJoints(self):
        return self.SO_filter.filter(self.q_des_gripper, self.gripping_duration)

    def move_gripper(self, diameter = 30, status = 'close'):
        # this is for the simulated robot, the diameter is converted into q for the fingers, that
        # will be appended to the desired joint published by the controller manager
        if not self.real_robot:
            q_finger = self.mapToGripperJoints(diameter)
            self.q_des_gripper = q_finger * np.ones(self.number_of_fingers)
            return


        # this is for the real robot, is a service call that sends a sting directly to the URcap driver
        import socket

        HOST = "192.168.0.100"  # The UR IP address
        PORT = 30002  # UR secondary client
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        sock.settimeout(0.5)
        try:
            sock.connect((HOST, PORT))
        except:
            raise Exception("Cannot connect to end-effector socket") from None
        sock.settimeout(None)
        scripts_path = rospkg.RosPack().get_path('ur_description') + '/gripper/scripts/'

        if self.gripper_type == 'soft_2':

            if diameter > 30.:
                script = scripts_path + 'soft_open.script'
            elif diameter < 30.:
                script = scripts_path + 'soft_close.script'
            else:
                script = scripts_path + 'soft_idle.script'

            f = open(script, "rb")
            l = f.read(2024)
            while (l):
                sock.send(l)
                l = f.read(2024)
            f.close()

        elif self.gripper_type == 'robotiq_2':
            # 2 finger rigid gripper
            robotiq_script = scripts_path + "/robotiq_set.script"

            file = open(robotiq_script, "rb")  # Robotiq Gripper
            lines = file.readlines()
            file.close()
            #0 open /255 closed
            input_robotiq = 255 - diameter*255/85.
            #print(input_robotiq)
            offset = 0
            buffer = 2500
            cmd_string1 = f'    rq_set_pos_spd_for({input_robotiq}, 255, 255, "1")\n'
            cmd_string2 = f'    rq_wait_pos_spe_for_request({input_robotiq}, 255, 255, "1")\n'
            line_number_to_add = 2422
            new_lines = lines[0:line_number_to_add]
            new_lines.insert(line_number_to_add + 1, str.encode(cmd_string1))
            new_lines.insert(line_number_to_add + 1, str.encode(cmd_string2))
            new_lines += lines[line_number_to_add::]
            # debug [b'    rq_activate_and_wait("1")\n', b'    rq_set_pos_spd_for(250, 255, 255, "1")\n', b'    rq_wait_pos_spe_for_request(250, 255, 255, "1")\n', b'    rq_go_to("1")\n', b'    rq_wait("1")\n', b'    # end: URCap Program Node\n', b'  end\n', b'end\n', b'\n', b'gripper_control()\n']
            # print(new_lines[-10:])
            lines1 = b''.join(new_lines)

            if len(lines1) < buffer:
                buffer = len(lines1)
            data = lines1[0:buffer]
            while data:
                sock.send(data)
                offset += buffer
                if len(lines1) < offset + buffer:
                    buffer = len(lines1) - offset
                data = lines1[offset:offset + buffer]
            sock.close()

        else:
            # 3 finger rigid gripper
            onrobot_script = scripts_path + "/onrobot_superminimal.script"
            file = open(onrobot_script, "rb")  # Robotiq Gripper
            lines = file.readlines()
            file.close()

            tool_index = 0
            blocking = True
            cmd_string = f"tfg_release({diameter},  tool_index={tool_index}, blocking={blocking})"

            line_number_to_add = 446

            new_lines = lines[0:line_number_to_add]
            new_lines.insert(line_number_to_add + 1, str.encode(cmd_string))
            new_lines += lines[line_number_to_add::]

            offset = 0
            buffer = 2024
            file_to_send = b''.join(new_lines)

            if len(file_to_send) < buffer:
                buffer = len(file_to_send)
            data = file_to_send[0:buffer]
            while data:
                sock.send(data)
                offset += buffer
                if len(file_to_send) < offset + buffer:
                    buffer = len(file_to_send) - offset
                data = file_to_send[offset:offset + buffer]
            sock.close()

        print("Gripper moved, now resend robot program")
        self.resend_robot_program()
        return

