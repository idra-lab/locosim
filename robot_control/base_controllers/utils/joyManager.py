#!/usr/bin/env python3
import rospy
import subprocess
from sensor_msgs.msg import Joy
import numpy as np
from base_controllers.utils.common_functions import checkRosMaster


class JoyManager:
    def __init__(self, port="js0"):
        """
        Manages joystick input by subscribing to /joy.
        Always restarts the joy_node to ensure a fresh connection.
        """
        self.port = port
        self.latest_msg = Joy()
        self.sub = rospy.Subscriber("/joy", Joy, self._joy_callback)
        self._restart_joy_node()

        rospy.loginfo("JoyManager initialized: subscribed to /joy")

    def _restart_joy_node(self):
        """
        Kills any existing joy_node and starts a new one.
        """
        rospy.logwarn("Restarting joy_node...")

        try:
            # Kill any old joy_node
            subprocess.run(["rosnode", "kill", "/joy", "/joy_node"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass  # ignore if no node exists

        # Start a new joy_node
        try:
            subprocess.Popen(["rosrun", "joy", "joy_node", "_dev:=/dev/input/"+self.port])
            rospy.loginfo("Started new joy_node instance.")
        except Exception as e:
            rospy.logerr(f"Failed to start joy_node: {e}")
            raise e

        # Wait until /joy topic is alive
        rospy.loginfo("Waiting for /joy messages...")
        try:
            rospy.wait_for_message("/joy", Joy, timeout=5.0)
            rospy.loginfo("Successfully connected to /joy.")
        except rospy.ROSException:
            rospy.logwarn("No /joy messages received within 5s. Check joystick connection.")

    def _joy_callback(self, msg):
        self.latest_msg = msg

    def get_commands(self):
        """
        Returns left and right stick values plus the four buttons (A, B, X, Y).
        """
        if not self.latest_msg.axes or not self.latest_msg.buttons:
            return np.zeros(4), np.zeros(4, dtype=int)

        try:
            # jessica joy
            # Right stick (move lateral)
            rot_z = self.latest_msg.axes[3]
            # Left stick (move forward/backward)
            long_x = self.latest_msg.axes[1]
            long_y = self.latest_msg.axes[0]
            graceful_shutdown = self.latest_msg.buttons[2]
            hard_shutdown = self.latest_msg.buttons[1]

            # focchi joy
            ## Left stick  (move lateral)
            # rot_z = self.latest_msg.axes[0]
            # # Right stick  (move forward/backward)
            # long_x = self.latest_msg.axes[3]
            # long_y = self.latest_msg.axes[2]
            # graceful_shutdown = self.latest_msg.buttons[0]
            # hard_shutdown = self.latest_msg.buttons[2]

            threshold = 0.05
            axes = np.array([
                long_x if abs(long_x) >= threshold else 0,
                long_y if abs(long_y) >= threshold else 0,
                rot_z if abs(rot_z) >= threshold else 0,
            ])

            # Digital buttons (X,Y,B,A)
            buttons = np.array([
                graceful_shutdown,
                hard_shutdown
            ], dtype=int)

            return axes, buttons

        except IndexError:
            return np.zeros(4), np.zeros(4, dtype=int)

    def get_start_button(self):
        """
        Returns True if safety button (Y on Xbox controller) is pressed.
        """
        try:
            return bool(self.latest_msg.buttons[9])
        except IndexError:
            return False


if __name__ == "__main__":
    checkRosMaster()
    rospy.init_node("joy_manager_node")
    jm = JoyManager()
    rate = rospy.Rate(10)

    rospy.loginfo("JoyManager node running. Listening to /joy...")
    print("Input\tIndex\tTypical Control\n"
          "axes[0]\t0\tLeft stick X\n"
          "axes[1]\t1\tLeft stick Y\n"
          "buttons[0]\t0\tA\n"
          "buttons[1]\t1\tB\n"
          "buttons[2]\t2\tX\n"
          "buttons[3]\t3\tY")

    while not rospy.is_shutdown():
        axes, buttons = jm.get_commands()
        rospy.loginfo_throttle(0.25,
                                f"axes: {jm.latest_msg.axes}, buttons: {jm.latest_msg.buttons}")
        rospy.loginfo_throttle(0.25, f"Axes: {axes}, Buttons: {buttons}, start: {jm.get_start_button():}")
        rate.sleep()

