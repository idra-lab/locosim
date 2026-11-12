cmake_minimum_required(VERSION 2.8.3)
project(ur_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY rviz config gripper sensors launch meshes urdf 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/go0
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

