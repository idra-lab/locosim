#ifndef CUSTOM_JOINT_PUB_H
#define CUSTOM_JOINT_PUB_H


#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>

typedef  Eigen::Matrix<double, 6, 1> JointStateVector;


// Methods
void send_des_jstate(const JointStateVector & joint_pos);
JointStateVector secondOrderFilter(const JointStateVector & input, const double rate, const double settling_time);

// Variables
JointStateVector q_des = JointStateVector::Zero();
JointStateVector q_des0 = JointStateVector::Zero();
JointStateVector qd_des = JointStateVector::Zero();
JointStateVector tau_ffwd = JointStateVector::Zero();
JointStateVector filter_1 = JointStateVector::Zero();
JointStateVector filter_2 = JointStateVector::Zero();

double  loop_time = 0.;
double  loop_frequency = 1000.;

// Publishers
//std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > pub_des_jstate_sim_rt;
ros::Publisher pub_des_jstate;
sensor_msgs::JointState jointState_msg_torque;
std_msgs::Float64MultiArray jointState_msg_position;

std::string control_type;

#endif
