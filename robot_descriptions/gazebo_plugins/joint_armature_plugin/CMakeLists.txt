cmake_minimum_required(VERSION 3.0.2)
project(joint_armature_plugin)

find_package(catkin REQUIRED COMPONENTS gazebo_ros)
find_package(gazebo REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(joint_armature_plugin SHARED src/joint_armature_plugin.cpp)
target_link_libraries(joint_armature_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# Next to the other locosim Gazebo plugins; the RL controller puts this directory on
# GAZEBO_PLUGIN_PATH before it starts the simulator.
install(TARGETS joint_armature_plugin DESTINATION $ENV{LOCOSIM_DIR}/robot_descriptions/gazebo_plugins/lib)
