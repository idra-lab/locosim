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

#f = open ("Grip.script", "rb")   #Robotiq Gripper
#f = open ("setzero.script", "rb")  #Robotiq FT sensor
scripts_path =  rospkg.RosPack().get_path('ur_description') + '/gripper/scripts/test_robotiq_gripper/'

def callback(data):
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

    print("Sending message")
    script = ''
    if (data.data == 'open'):
        robotiq_script = scripts_path + 'robotiq_open.script'
        print("open")
    elif (data.data == 'close'):
        robotiq_script = scripts_path + 'robotiq_close.script'
        print("close")
    elif (data.data == 'set'):#TODO
        robotiq_script = scripts_path + 'robotiq_set.script'
        print("set ")
    else:
        print("Invalid argument!")


    file = open(robotiq_script, "rb")  # Robotiq Gripper
    lines = file.readlines()
    file.close()
    offset = 0
    buffer = 2500

    if (data.data == 'set'):
        diameter = 120
        cmd_string1 = f'    rq_set_pos_spd_for({diameter}, 255, 255, "1")\n'
        cmd_string2 = f'    rq_wait_pos_spe_for_request({diameter}, 255, 255, "1")\n'
        line_number_to_add = 2422
        new_lines = lines[0:line_number_to_add]
        new_lines.insert(line_number_to_add + 1, str.encode(cmd_string1))
        new_lines.insert(line_number_to_add + 1, str.encode(cmd_string2))
        new_lines += lines[line_number_to_add::]
        #debug [b'    rq_activate_and_wait("1")\n', b'    rq_set_pos_spd_for(250, 255, 255, "1")\n', b'    rq_wait_pos_spe_for_request(250, 255, 255, "1")\n', b'    rq_go_to("1")\n', b'    rq_wait("1")\n', b'    # end: URCap Program Node\n', b'  end\n', b'end\n', b'\n', b'gripper_control()\n']
        #print(new_lines[-10:])
        lines1 = b''.join(new_lines)
    else:
        # with only oper close
        #debug check last commands [b'    rq_activate_and_wait("1")\n', b'    rq_set_pos_spd_for(255, 255, 255, "1")\n', b'    rq_wait_pos_spe_for_request(255, 255, 255, "1")\n', b'    rq_go_to("1")\n', b'    rq_wait("1")\n', b'    # end: URCap Program Node\n', b'  end\n', b'end\n', b'\n', b'gripper_control()\n']
        #print(lines[-10:])
        lines1 = b''.join(lines)

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

    resend_robot_program()


def resend_robot_program():
    ros.sleep(1.5)
    sos_service = ros.ServiceProxy('/ur5/ur_hardware_interface/resend_robot_program', Trigger)
    sos = TriggerRequest()
    result = sos_service(sos)
    # print(result)
    ros.sleep(1.0)

def listener():
    rospy.init_node('gripper_controller_execution')

    ##### TO USE in another terminal launch: rostopic pub /gripper_controller_cmd std_msgs/String "data: '"open/close"'"

    rospy.Subscriber("/gripper_controller_cmd", String, callback,queue_size=10)
    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
if __name__ == '__main__':
    import os
    os.system("rosnode kill /gripper_controller_execution")
    listener()

