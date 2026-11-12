# import psutil
# import signal
# import subprocess
# import os
# import time
#
# def terminate_ros_node(p):
#   process = psutil.Process(p.pid)
#   for sub_process in process.get_children(recursive=True):
#       sub_process.send_signal(signal.SIGINT)
#   p.wait()  # we wait for children to terminate
#   p.terminate()
#
# def terminate_ros_node(s):
#   list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
#   list_output = list_cmd.stdout.read()
#   print(list_output)
#   retcode = list_cmd.wait()
#   assert retcode == 0, "List command returned %d" % retcode
#   for str in list_output.split("\n"):
#     if (str.startswith(s)):
#       os.system("rosnode kill " + str)
#
# if __name__ == '__main__':
#   dir_save_bagfile = '/home/mfocchi/'
#   rosbag_process = subprocess.Popen('rosbag record -a -j -o {}'.format("myrosbag"), stdin=subprocess.PIPE, shell=True, cwd=dir_save_bagfile)
#   time.sleep(2.5)
#   print("stopping rosbag")
#   terminate_ros_node(rosbag_process)

#!/usr/bin/env python

"""Node to record a rosbag with start/stop/pause control through service calls.
Example call:
    rosrun utilities rosbag_controlled_recording.py _rosbag_command:="rosbag record -o /home/foo/test_bag /bar_topic" _record_from_startup:=false
Then start/pause/resume/stop can be controlled through:
    rosservice call /rosbag_controlled_recording/start
    rosservice call /rosbag_controlled_recording/pause_resume
    rosservice call /rosbag_controlled_recording/pause_resume
    rosservice call /rosbag_controlled_recording/stop
Note that pausing does not modify the recorded time of messages, i.e. the bag's total length is unaffected. A list of
  pause-resume times is logged when stopping, in case the paused period needs to be (manually) removed afterwards.
If this node is killed recording is also stopped. If recording was paused, it is momentarily resumed before stopping.
"""

import psutil
import subprocess
import shlex
import signal
import time
import rospy
from termcolor import colored
from std_srvs.srv import Empty, EmptyResponse
from base_controllers.utils.common_functions import checkRosMaster
import os

def _pid_exists(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False

def signal_process_group(pgid: int, sig: int, wait_pids=None, timeout_s: float = 10.0):
    """Send signal to a process group and optionally wait for processes to exit."""
    try:
        os.killpg(pgid, sig)
    except ProcessLookupError:
        return

    if not wait_pids:
        return

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        alive = False
        for pid in wait_pids:
            if pid and _pid_exists(pid):
                alive = True
                break
        if not alive:
            return
        time.sleep(0.05)



def format_to_columns(input_list, cols):
    """Adapted from https://stackoverflow.com/questions/171662/formatting-a-list-of-text-into-columns"""
    max_width = max(map(len, input_list))
    justify_list = map(lambda x: x.ljust(max_width + 4), input_list)
    lines = (''.join(justify_list[i:i + cols]) for i in range(0, len(justify_list), cols))
    return '\n'.join(lines)

class RosbagControlledRecorder(object):
    """Record a rosbag with service calls to control start, stop and pause."""

    def __init__(self, topics=' -a', bag_name=None, record_from_startup_=False, bag_folder=None, local_folder=False):
        rosbag_command_ = "rosbag record " + topics
        self.bag_name = bag_name
        self.bag_folder = bag_folder

        self.full_path = None
        if self.bag_name is not None:
            if self.bag_folder is not None:
                if local_folder:
                    self.full_path = os.path.join(os.getcwd(), self.bag_folder, self.bag_name)
                else:
                    self.full_path = f"{self.bag_folder.rstrip('/')}/{self.bag_name}"
                os.makedirs(os.path.dirname(self.full_path), exist_ok=True)
            else:
                self.full_path = self.bag_name
            rosbag_command_ += " -O " + self.full_path

        self.rosbag_command = shlex.split(rosbag_command_)

        self.recording_started = False
        self.recording_paused = False
        self.recording_stopped = False
        self.pause_resume_times = []

        # IMPORTANT: keep the Popen handle, not only PID
        self._proc = None
        self._pgid = None

        if record_from_startup_:
            self.start_recording_srv()

        self.exit_loop = False


    def start_recording_srv(self, service_message=None):
        if self._proc is not None and self._proc.poll() is None:
            rospy.logwarn("rosbag record already running; ignoring start")
            return EmptyResponse()

        print(colored(f"Starting Bag {self.bag_name}", "red"))

        # Start in its own process group so SIGINT behaves like Ctrl-C for rosbag
        self._proc = subprocess.Popen(
            self.rosbag_command,
            preexec_fn=os.setsid,        # new process group
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        self._pgid = os.getpgid(self._proc.pid)

        self.recording_started = True
        self.recording_paused = False
        self.recording_stopped = False
        self.pause_resume_times = []

        rospy.loginfo("Started recording rosbag (pid=%d, pgid=%d)", self._proc.pid, self._pgid)
        return EmptyResponse()

    def pause_resume_recording_srv(self, service_message=None):
        if not self.recording_started or self._proc is None or self._proc.poll() is not None:
            rospy.logwarn("Recording not running - nothing to be done")
            return EmptyResponse()

        try:
            p = psutil.Process(self._proc.pid)
        except psutil.NoSuchProcess:
            rospy.logwarn("Recorder process disappeared")
            return EmptyResponse()

        if self.recording_paused:
            # resume
            for ch in p.children(recursive=True):
                ch.resume()
            p.resume()
            self.recording_paused = False
            rospy.loginfo("Recording resumed")
        else:
            # suspend
            for ch in p.children(recursive=True):
                ch.suspend()
            p.suspend()
            self.recording_paused = True
            rospy.loginfo("Recording paused")

        self.pause_resume_times.append(rospy.get_time())
        return EmptyResponse()

    def stop_recording_srv(self, service_message=None):
        print(colored("Saving Bag...wait!", "red"))

        if self._proc is None:
            self.recording_stopped = True
            return EmptyResponse()

        if self._proc.poll() is not None:
            # already exited
            self._proc = None
            self._pgid = None
            self.recording_stopped = True
            return EmptyResponse()

        # If paused, resume before stopping so rosbag can close cleanly
        if self.recording_paused:
            self.pause_resume_recording_srv()
            rospy.sleep(0.2)

        # Log pause/resume times (fix Python3 map issue)
        if self.pause_resume_times:
            pause_resume_str = list(map(str, self.pause_resume_times))
            pause_resume_str[0:0] = ['PAUSE', 'RESUME']
            rospy.loginfo("List of pause and resume times:\n%s\n", format_to_columns(pause_resume_str, 2))

        # Send SIGINT like Ctrl-C, to the WHOLE process group
        try:
            pgid = self._pgid if self._pgid is not None else os.getpgid(self._proc.pid)
            signal_process_group(pgid, signal.SIGINT, wait_pids=[self._proc.pid], timeout_s=15.0)
        except Exception as e:
            rospy.logwarn("Failed SIGINT to rosbag process group: %s", e)

        # Wait on Popen to ensure clean close + index write
        try:
            self._proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            rospy.logwarn("rosbag did not exit after SIGINT; sending SIGTERM")
            try:
                pgid = os.getpgid(self._proc.pid)
                signal_process_group(pgid, signal.SIGTERM, wait_pids=[self._proc.pid], timeout_s=1.0)
                self._proc.wait(timeout=1.0)
            except Exception:
                rospy.logwarn("rosbag still alive; sending SIGKILL (may corrupt index)")
                try:
                    pgid = os.getpgid(self._proc.pid)
                    os.killpg(pgid, signal.SIGKILL)
                except Exception:
                    pass

        # small flush delay (helps on some FS / docker volumes)
        time.sleep(0.5)

        # Read stdout/stderr (optional, but useful for debugging)
        try:
            out, err = self._proc.communicate(timeout=0.1)
            if err:
                rospy.logdebug("rosbag stderr:\n%s", err)
        except Exception:
            pass

        self._proc = None
        self._pgid = None

        self.recording_started = False
        self.recording_paused = False
        self.recording_stopped = True

        rospy.loginfo("Stopped recording rosbag")
        return EmptyResponse()

    def rosbagPlay(self, bag_file, robot_name="tractor", upload_args='', bag_options=''):
        """Play the rosbag, setup Ctrl+C handling, and allow early termination."""
        print(colored(f"Starting rosbag play for {bag_file}", "green"))

        # Setup signal handler for Ctrl + C (SIGINT)
        signal.signal(signal.SIGINT, self.stop_loop_early)

        # Command to launch rviz and play the rosbag
        command = (
            "pkill rviz & "
            f"roslaunch "+robot_name+"_description upload.launch "+upload_args+"  & "
            "rosrun rviz rviz -d $LOCOSIM_DIR/robot_descriptions/"+robot_name+"_description/rviz/conf.rviz & "
            f"rosbag play {bag_file}  "+bag_options
        )
        self.rosbag_play_process = subprocess.Popen(command, shell=True, executable="/bin/bash")

        # Main loop that checks for premature exit or completion
        while not rospy.is_shutdown():
            if self.exit_loop:  # If Ctrl + C was pressed, exit loop
                print("Premature loop exit due to Ctrl+C")
                break
            rospy.sleep(1.0)
        # After the loop ends, ensure the subprocess is terminated
        if self.rosbag_play_process:
            self.kill_process_and_children(self.rosbag_play_process)

        print("Playback completed.")

    def stop_loop_early(self, signum, frame):
        """Exit the loop on Ctrl+C (SIGINT)."""
        self.exit_loop = True
        print("Received Ctrl+C. Exiting the loop early.")
        rospy.signal_shutdown("Received SIGINT")

    def kill_process_and_children(self, process):
        """Terminate the given process and all of its children."""
        try:
            # Get the parent process ID (PID) of the rosbag play process
            p = psutil.Process(process.pid)

            # Kill all child processes of rosbag play process first
            for child in p.children(recursive=True):
                child.kill()  # Send SIGKILL to child processes

            # Then kill the main rosbag play process
            p.kill()  # Send SIGKILL to the main process

            print(f"Killed process and its children (PID: {process.pid})")
        except psutil.NoSuchProcess:
            print("Process already terminated.")
        except Exception as e:
            print(f"Failed to kill process: {e}")

if __name__ == '__main__':
    checkRosMaster()
    rospy.init_node('rosbag_controlled_recording')

    # Get parameters

    # Start recorder object
    recorder = RosbagControlledRecorder(bag_name="prova.bag", bag_folder="test_bag", local_folder=True)

    # Services
    start_service = rospy.Service('~start', Empty, recorder.start_recording_srv)
    pause_resume_service = rospy.Service('~pause_resume', Empty, recorder.pause_resume_recording_srv)
    stop_service = rospy.Service('~stop', Empty, recorder.stop_recording_srv)

    # Recording is also stopped on node shutdown. This allows stopping to be done via service call or regular Ctrl-C
    rospy.on_shutdown(recorder.stop_recording_srv)
    print("start 1st bag")
    time.sleep(2)
    recorder.start_recording_srv()
    time.sleep(5)
    recorder.stop_recording_srv()
    print("start 2nd bag")
    time.sleep(10)
    recorder.start_recording_srv()
    time.sleep(10)
    recorder.stop_recording_srv()


    while not rospy.is_shutdown():
        if recorder.recording_stopped:  # stop main node if recording has finished
            print("finish")
            break
        rospy.sleep(1.0)


