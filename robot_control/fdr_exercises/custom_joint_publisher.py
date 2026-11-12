#!/usr/bin/env python
import rospy as ros
import numpy as np
from std_msgs.msg import Float64MultiArray


class JointStatePublisher():

    def __init__(self):
        self.q_des =np.zeros(6)
        self.qd_des = np.zeros(6)
        self.tau_ffwd = np.zeros(6)


    def send_des_jstate(self, q_des):
        msg = Float64MultiArray()
        if (self.gripper_sim):
            msg.data = np.append(q_des, np.array([0.0, 0.0 ,0.0]))
        else:
            msg.data = q_des
        self.pub_des_jstate.publish(msg)

def talker(p):
    ros.init_node('custom_joint_pub_node', anonymous=True)
    p.pub_des_jstate = ros.Publisher("/ur5/joint_group_pos_controller/command", Float64MultiArray, queue_size=1)
    loop_frequency = 1000.
    loop_rate = ros.Rate(loop_frequency)  # 1000hz

    # init variables
    time = 0
    q_des0 = np.array([ -0.32,-0.78, -2.56,-1.63, -1.57, 3.49])

    # 1 fixed setpoint
    p.q_des = q_des0

    # check if gripper is actuated
    p.gripper_sim = ros.get_param("/gripper_sim")

    amp = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0])  # amplitude
    freq = np.array([0.2, 0.0, 0.0, 0.0, 0., 0.0]) # frequency

    while not ros.is_shutdown():
        if p.gripper_sim == True:
            print("this publisher cannot handle the gripper joints")
            break
        # 2 - generate step reference
<<<<<<< HEAD:lab_exercises/lab_palopoli/custom_joint_publisher.py
        if time < 4.:
            p.q_des = q_des0
        else:
            p.q_des = q_des0 + np.array([0., 0.4, 0., 0., 0., 0])
=======
        # if time < 4.:
        #     p.q_des = q_des0
        # else:
            #p.q_des = q_des0 + np.array([0., 0.4, 0., 0., 0., 0])
>>>>>>> 05fc729 (renaming exercise folders):fdr_exercises/custom_joint_publisher.py


        p.qd_des = np.zeros(6)
        p.tau_ffwd = np.zeros(6)

        p.send_des_jstate(p.q_des)

        time = np.round(time + np.array([1/loop_frequency]), 3)
        loop_rate.sleep()

if __name__ == '__main__':
    myPub = JointStatePublisher()
    try:
        talker(myPub)
    except ros.ROSInterruptException:
        pass
