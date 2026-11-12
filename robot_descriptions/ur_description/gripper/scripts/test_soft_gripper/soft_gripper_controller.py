#!/usr/bin/env python 
# Echo client program
import socket
import sys
import os
import rospy
import rospkg
import rospy as ros
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import String
HOST = "192.168.0.100" # The UR IP address
PORT = 30002 # UR secondary client
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
#f = open ("Grip.script", "rb")   #Robotiq Gripper
#f = open ("setzero.script", "rb")  #Robotiq FT sensor
path = scripts_path = rospkg.RosPack().get_path('ur_description') + '/gripper/scripts/test_soft_gripper/'

def callback(data):
    
    script=''
    if(data.data == 'open1'):
        script = path + 'open1.script'
        print("open1")
    elif(data.data == 'open2'):
        script = path + 'open2.script'
        print("open2")
    elif(data.data == 'close'):
        script = path + 'close.script'
        print("close")
    else:
        print("Invalid argument!")

    f = open (script, "rb")
    l = f.read(2024)
    while (l):
        s.send(l)
        l = f.read(2024)
    f.close()
    resend_robot_program()


def resend_robot_program():
    ros.sleep(1.5)
    sos_service = ros.ServiceProxy('/ur5/ur_hardware_interface/resend_robot_program', Trigger)
    sos = TriggerRequest()
    result = sos_service(sos)
    # print(result)
    ros.sleep(1.0)

def listener():
    rospy.init_node('gripper_controller_execution', anonymous=True)


    rospy.Subscriber("/gripper_controller_cmd", String, callback,queue_size=10)
    rospy.spin()
    ##### TOUSE rostopic pub /gripper_controller_cmd std_msgs/String "data: '"open/close"'"


    # spin() simply keeps python from exiting until this node is stopped
if __name__ == '__main__':
    listener()

