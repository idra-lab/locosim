#!/usr/bin/env python
import rospy as ros
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JointStatePublisher():

    def __init__(self):
        self.q_des =np.zeros(6)
        self.qd_des = np.zeros(6)
        self.tau_ffwd = np.zeros(6)

        #filter variables
        self.filter_1 = np.zeros(6)
        self.filter_2 = np.zeros(6)

        self.q =np.zeros(6)
        self.joint_names =  ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    # subscriber callback
    def receive_jstate(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.q[joint_idx] = msg.position[msg_idx]

    def initFilter(self, q):
        self.filter_1 = np.copy(q)
        self.filter_2 = np.copy(q)

    def secondOrderFilter(self, input, rate, settling_time):
        dt = 1 / rate
        gain =  dt / (0.1*settling_time + dt)
        self.filter_1 = (1 - gain) * self.filter_1 + gain * input
        self.filter_2 = (1 - gain) * self.filter_2 + gain * self.filter_1
        return self.filter_2

    def send_des_jstate(self):
        msg = Float64MultiArray()
        # check if gripper is actuated if it is just append zeros
        if (self.gripper_sim):
            msg.data = np.append(self.q_des, np.array([0.0, 0.0 ,0.0]))
        else:
            msg.data = self.q_des
        self.pub_des_jstate.publish(msg)

def talker(p):
    ros.init_node('custom_joint_pub_node', anonymous=True)
    p.real_robot = ros.get_param("real_robot")
    if p.real_robot:
        print("Robot is REAL")

    p.pub_des_jstate = ros.Publisher("/ur5/joint_group_pos_controller/command", Float64MultiArray, queue_size=1)
    p.sub_jstate = ros.Subscriber("/ur5/joint_states", JointState, callback = p.receive_jstate, queue_size=1)

    ros.sleep(2.)
    loop_frequency = 1000.
    loop_rate = ros.Rate(loop_frequency)  # 1000hz

    # init variables
    time = 0.
    print("init q: ",p.q)

    #initialize filter with actual values
    q0 = np.copy(p.q)
    p.initFilter(q0)

    # check if gripper is actuated
    p.gripper_sim = ros.get_param("/gripper_sim")

    amp = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0])  # amplitude
    freq = np.array([0.2, 0.0, 0.0, 0.0, 0., 0.0]) # frequency


    while not ros.is_shutdown():
        # 3- generate filtered step reference
        # if time < 4.:
        #     p.q_des = q0
        # else:
        #     p.q_des = p.secondOrderFilter(q0 + np.array([0., 0.4, 0., 0., 0., 0]), loop_frequency, 5.)

        # 4 - generate sine reference
        amp = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0])  # amplitude
        freq = np.array([0.2, 0.0, 0.0, 0.0, 0., 0.0])  # frequency
        p.q_des = q0 + np.multiply(amp, np.sin(2 * np.pi * freq * time))

        p.qd_des = np.zeros(6)
        p.tau_ffwd = np.zeros(6)
        print(p.q_des)
        p.send_des_jstate()
        time = np.round(time + np.array([1/loop_frequency]), 3)
        loop_rate.sleep()

if __name__ == '__main__':
    myPub = JointStatePublisher()
    try:
        talker(myPub)
    except ros.ROSInterruptException:
        pass
