/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef __MULTISENSE_SL_PLUGIN_HH_
#define __MULTISENSE_SL_PLUGIN_HH_

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

#include <std_srvs/Empty.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/MultiCameraSensor.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/Sensor.hh>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class MultiSenseSL : public ModelPlugin
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model
    public: MultiSenseSL();

    /// \brief Destructor
    public: ~MultiSenseSL();

    /// \brief Load the plugin
    /// \param[in] _parent pointer to parent Model
    /// \param[in] _sdf SDF root element corresponds to the plugin XML block
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller periodically via Events.
    protected: virtual void UpdateStates();

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief Thread for loading and initializing ROS
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;

    // IMU sensor
    private: std::shared_ptr<sensors::ImuSensor> imuSensor;
    private: std::string imuLinkName;
    private: physics::LinkPtr imuLink;
    private: ros::Publisher pubImu;
    private: PubQueue<sensor_msgs::Imu>::Ptr pubImuQueue;

    // reset of ros stuff
    private: ros::NodeHandle* rosnode_;
    private: ros::CallbackQueue queue_;
    private: void QueueThread();
    private: boost::thread callback_queue_thread_;

    // ros topic subscriber
    private: ros::Subscriber set_spindle_speed_sub_;
    private: void SetSpindleSpeed(const std_msgs::Float64::ConstPtr &_msg);

    private: ros::Subscriber set_spindle_state_sub_;
    private: void SetSpindleState(const std_msgs::Bool::ConstPtr &_msg);

    /// for deprecation from ~/multisense_sl/fps to ~/multisense_sl/set_fps
    /// per issue 272
    private: ros::Subscriber set_multi_camera_frame_rate_sub_old_;
    private: void SetMultiCameraFrameRateOld(const std_msgs::Float64::ConstPtr
                                         &_msg);

    private: ros::Subscriber set_multi_camera_frame_rate_sub_;
    private: void SetMultiCameraFrameRate(const std_msgs::Float64::ConstPtr
                                         &_msg);

    private: ros::Subscriber set_multi_camera_resolution_sub_;

    private: std::string rosNamespace;

    /// \brief camera resolution modes
    /// available modes are:
    ///  0 - 2MP (2048*1088) @ up to 15 fps
    ///  1 - 1MP (1536*816) @ up to 30 fps
    ///  2 - 0.5MP (1024*544) @ up to 60 fps (default)
    ///  3 - VGA (640*480) @ up to 70 fps
    private: void SetMultiCameraResolution(
      const std_msgs::Int32::ConstPtr &_msg);

    private: ros::Subscriber set_multi_camera_exposure_time_sub_;
    private: void SetMultiCameraExposureTime(const std_msgs::Float64::ConstPtr
                                            &_msg);

    private: ros::Subscriber set_multi_camera_gain_sub_;
    private: void SetMultiCameraGain(const std_msgs::Float64::ConstPtr
                                    &_msg);

    // ros services
    private: ros::ServiceServer set_spindle_state_service_;
    private: bool SetSpindleState(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    private: ros::ServiceServer set_spindle_speed_service_;
    private: bool SetSpindleSpeed(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res);

    // gazebo variables
    private: physics::WorldPtr world;
    private: physics::ModelPtr robotModel;
    private: sdf::ElementPtr sdf;
    private: common::Time lastTime;

    // joint state
    private: ros::Publisher pubJointStates;
    private: PubQueue<sensor_msgs::JointState>::Ptr pubJointStatesQueue;
    private: sensor_msgs::JointState jointStates;

    // camera control
    private: std::shared_ptr<sensors::MultiCameraSensor> multiCameraSensor;
    private: double multiCameraFrameRate;
    private: double multiCameraExposureTime;
    private: double multiCameraGain;
    private: int imagerMode;

    // spindle control
    private: double spindleSpeed;
    private: double spindleMaxRPM;
    private: double spindleMinRPM;
    private: bool spindleOn;
    private: physics::LinkPtr spindleLink;
    private: physics::JointPtr motor_joint;
    private: physics::JointPtr spindle_joint; // fake joint (static)
    private: physics::JointPtr camera_joint;  // fake joint (static)
    private: common::PID spindlePID;

    /// Throttle update rate
    private: double lastUpdateTime;
    private: double updateRate;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue* pmq;
    tf::TransformBroadcaster static_tf_broadcaster_;

    tf::Transform motor_to_camera_;
    tf::Transform laser_to_spindle_;

    //
    // Frames to Publish
    std::string left_camera_optical_;
    std::string motor_;
    std::string spindle_;
    std::string hokuyo_;
  };
}
#endif
