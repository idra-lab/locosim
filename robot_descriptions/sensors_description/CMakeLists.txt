cmake_minimum_required(VERSION 3.1)
project(sensors_description)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             sensor_msgs
             gazebo_ros
             geometry_msgs
             gazebo_plugins
             image_transport
             camera_info_manager
             tf
)

find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drcsim_gazebo_ros_plugins
  CATKIN_DEPENDS gazebo_plugins
#  DEPENDS system_lib
)

find_package(PkgConfig)
pkg_check_modules(OGRE OGRE)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}/Paging
  ${OGRE_INCLUDE_DIRS}/Terrain
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Declare a cpp library
add_library(MultiSenseSLPlugin src/MultiSenseSLPlugin.cpp)
#target_link_libraries(MultiSenseSLPlugin ${catkin_LIBRARIES})
add_library(RealsenseGazeboPlugin
    src/RealSensePlugin.cpp
    src/gazebo_ros_realsense.cpp)
target_link_libraries(RealsenseGazeboPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS MultiSenseSLPlugin RealsenseGazeboPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY urdfs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdfs
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
