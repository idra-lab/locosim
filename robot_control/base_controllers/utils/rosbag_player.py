from base_controllers.utils.common_functions import checkRosMaster
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
import os
import rospy
from termcolor import colored

if __name__ == '__main__':
    checkRosMaster()
    rospy.init_node('rosbag_controlled_recording')
    recorder = RosbagControlledRecorder()
    bag_location = os.environ["LOCOSIM_DIR"]+"/robot_control/base_controllers/"
    #bag_name = "saferl3_withoutcrane.bag"
    bag_name = "saferl_2026-02-25-11-42-29.bag"

    if not os.path.exists(bag_location+bag_name):
        print(colored("Bag File does not exists.", "red"))
    else:
        recorder.rosbagPlay(robot_name="aliengo", bag_file=bag_location+bag_name, bag_options="-r 1 -s 2 ")


