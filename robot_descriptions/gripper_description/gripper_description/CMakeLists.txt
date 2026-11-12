cmake_minimum_required(VERSION 2.8.3)
project(gripper_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY   launch rviz meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

