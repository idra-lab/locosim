#!/usr/bin/env python
import rospy
import tf
import numpy as np
import tf.transformations as tr
import roslaunch
import rospkg
import os
import time

if __name__ == '__main__':
    os.system("killall rosmaster rviz")

    # launch zed wrapper
    print("STARTING ZED WRAPPER")
    uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
    roslaunch.configure_logging(uuid)
    launch_file = rospkg.RosPack().get_path('zed_wrapper') + '/launch/zed2.launch'
    cli_args = [launch_file, 'rviz:=true']
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    zed_wrapper = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file, is_core=True)
    zed_wrapper.start()
    rospy.sleep(8.)

    print("STARTING ARUCO")
    # # launch aruco ros
    uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
    roslaunch.configure_logging(uuid)
    launch_file = rospkg.RosPack().get_path('aruco_ros') + '/launch/single.launch'
    cli_args = [launch_file]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0])]
    aruco_ros = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    aruco_ros.start()
    rospy.sleep(8.)

    print("STARTING CAMERA CALIBRARION")
    rospy.init_node('camera_calibration', anonymous=False)

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    # aruco marker pose location wrt WF
    # rotation about Z of  -90
    w_R_q = np.array([[0 , 1 ,0 ], [-1 , 0 ,0 ], [0 , 0 ,1 ]])
    w_t_q = np.array([0.055,  0.795, 0.852])
    w_t_b = np.array([0.5, 0.35, 1.75])
    #
    # aruco marker pose location wrt BF
    b_t_q = w_t_q -w_t_b
    b_R_q = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])

    while not rospy.is_shutdown():
        try:
            # the origin of base_link in the zed wrapper corresponds to the origin of the zed2_camera_center
            (q_t_c,quat) = listener.lookupTransform('/aruco_marker_frame', '/root_camera_frame',rospy.Time(0))
            #print("TF:translation in zed2_left_camera_optical_frame :", q_t_c)
            #print("TF:rotation in zed2_left_camera_optical_frame:", quat)
            q_R_c = tr.quaternion_matrix(quat)[0:3,0:3]

            #map to WF
            # w_R_c = w_R_q.dot(q_R_c)
            # w_t_c = w_t_q + w_R_q.dot(q_t_c)
            # print("TF camera translation wrt WF:", w_t_c)
            # print("TF camera rotation wrt WF:", w_R_c)
            # yaw, pitch, roll = tr.euler_from_matrix(w_R_c, 'rzyx')

            # map to BF (we use this cause we refer to BF because we spawn in Gazebo)
            b_R_c = b_R_q.dot(q_R_c)
            b_t_c = b_t_q + b_R_q.dot(q_t_c)
            print("TF camera translation wrt BF:", b_t_c)
            #print("TF camera rotation wrt WF:", b_R_c)
            yaw, pitch, roll = tr.euler_from_matrix(b_R_c, 'rzyx')
            print('roll', roll)
            print('pitch', pitch)
            print('yaw', yaw)
            # damp into Yaml file TODO
            os.system("killall rosmaster rviz")
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
