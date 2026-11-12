
#!/usr/bin/env python

from __future__ import print_function
import rospy as ros
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import ros_numpy
import numpy as np
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 10000, suppress = True)
import struct
import pcl
import pcl.pcl_visualization

import matplotlib.pyplot as plt
import math

#Resources

# native reading best explanation
#https: // answers.ros.org / question / 219876 / using - sensor_msgspointcloud2 - natively /
# https://medium.com/@jeehoahn/some-ideas-on-pointcloud2-data-type-1d1ae940ef9b
# https://answers.ros.org/question/373094/understanding-pointcloud2-data/
# https://robotics.stackexchange.com/questions/19290/what-is-the-definition-of-the-contents-of-pointcloud2


def receive_pointcloud(msg):
    # read all the points
    #array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)

    # get the lowest distance
    # distance_array = np.zeros(shape=(array.shape[0], 1))
    # for i in range(array.shape[0]):
    #     distance = math.sqrt(array[i, 0] ** 2 + array[i, 1] ** 2 + array[i, 2] ** 2)
    #     # print(distance)
    #     distance_array[i, 0] = distance
    # print("Minimum distance  is: " + str(np.min(distance_array)))
    # min_index = np.argmin(distance_array)
    # print("Minimum distance index is: " + str(np.argmin(distance_array)))
    # print("position  of min distance point is: " + str(array[min_index, :]))

    #get one (or more) points from the pointcloud (unfortunately you need an iterator)
    # These X,Y,Z values are in the zed2_left_camera_optical_frame (X is seen as going to left (looking the camera)
    #  Y is top to bottom and Z pointing out the camera). You can select the pixel in the image frame with the u,v variable,
    #  u = 0 , v = 0 is the top right corner, u = 640 , v = 360 is the middle of the image plane which corresponds to the origin of the zed2_left_camera_optical_frame
    points_list = []
    for data in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=False, uvs=[(640, 360)]):
        points_list.append([data[0], data[1], data[2]])
    print("Data: ", points_list)



if __name__ == '__main__':
    ros.init_node('custom_joint_pub_node', anonymous=True)
    sub_pointcloud = ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, callback = receive_pointcloud, queue_size=1)

    loop_rate = ros.Rate(1.)
    while True:
        loop_rate.sleep()
        pass
