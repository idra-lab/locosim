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
import rospy

import socket


def resend_robot_program():
    ros.sleep(1.5)
    sos_service = ros.ServiceProxy('/ur5/ur_hardware_interface/resend_robot_program', Trigger)
    sos = TriggerRequest()
    result = sos_service(sos)
    # print(result)
    ros.sleep(1.0)


if __name__ == '__main__':
    rospy.init_node('gripper_controller_execution', anonymous=True)
    HOST = "192.168.0.100"  # The UR IP address
    PORT = 30002  # UR secondary client
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.5)
    try:
        s.connect((HOST, PORT))
    except:
        raise Exception("Cannot connect to end-effector socket") from None
    s.settimeout(None)
    scripts_path = rospkg.RosPack().get_path('ur_description') + '/gripper/scripts/test_gripper/'

    diameter = 20
    if diameter>30.:
        script = scripts_path + 'open2.script'
    else:
        script = scripts_path + 'close.script'

    f = open (script, "rb")
    l = f.read(2024)
    while (l):
        s.send(l)
        l = f.read(2024)
    f.close()
    resend_robot_program()
