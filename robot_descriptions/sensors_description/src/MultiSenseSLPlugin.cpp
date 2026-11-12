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

#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/rendering/Camera.hh>
#include <sensor_msgs/Imu.h>

#include "sensors_description/MultiSenseSLPlugin.h"

#include <ignition/math.hh>


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MultiSenseSL)

////////////////////////////////////////////////////////////////////////////////
MultiSenseSL::MultiSenseSL()
{
  /// \todo: hardcoded for now, make them into plugin parameters
  this->spindlePID.Init(0.03, 0.30, 0.00001, 1., -1., 10.0, -10.0);
  this->spindleOn = true;
  this->spindleSpeed = 0;
  this->spindleMaxRPM = 50.0;
  this->spindleMinRPM = 0;
  this->multiCameraExposureTime = 0.001;
  this->multiCameraGain = 1.0;
  // the parent link of the head_imu_sensor ends up being head after
  // fixed joint reduction.  Offset of the imu_link is lumped into
  // the <pose> tag in the imu_sensor block.
  this->imuLinkName = "base_link";

  // change default imager mode to 1 (1Hz ~ 30Hz)
  // in simulation, we are using 800X800 pixels @30Hz
  this->imagerMode = 1;
  this->rosNamespace = "/multisense";

  this->pmq = new PubMultiQueue();

    // These two transforms are fixed and taken from internal calibration of the
    // multisense. They should be equal to the origin of the fake joints
    // defined in the multisense_sl.urdf.xacro file,
    // even though they are in RPY format there.
    this->motor_to_camera_.setOrigin(tf::Vector3(0.027155533433,
                                                 -0.0971753746271,
                                                 -0.0172099247575));

    this->motor_to_camera_.setRotation(tf::Quaternion(-0.00284684825362,
                                                      0.00284684825362,
                                                      0.858563220921,
                                                      0.512691882894));

    this->laser_to_spindle_.setOrigin(tf::Vector3(0.000662488571834,
                                                  0.00996151566505,
                                                  -7.18757200957e-06));

    this->laser_to_spindle_.setRotation(tf::Quaternion(-0.00111727367235,
                                                       -0.0113784726912,
                                                       -1.27136996881e-05,
                                                       0.999934641371));

    this->left_camera_optical_ =   "multisense/left_camera_optical_frame";
    this->motor_ = "multisense/motor";
    this->spindle_ = "multisense/spindle";
    this->hokuyo_ = "multisense/hokuyo_link";

}

////////////////////////////////////////////////////////////////////////////////
MultiSenseSL::~MultiSenseSL()
{
  //event::Events::DisconnectWorldUpdateBegin(this->updateConnection);<-- NOT SURE IF THIS CAN BE REMOVED!
  delete this->pmq;
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->robotModel = _parent;
  this->world = _parent->GetWorld();
  this->sdf = _sdf;


  std::string prefix = "multisense/";
  ROS_DEBUG("Loading Multisense ROS node.");

  this->lastTime = this->world->SimTime();

  // Get imu link
  this->imuLink = this->robotModel->GetLink(this->imuLinkName);
  if (!this->imuLink)
  {
    gzerr << this->imuLinkName << " not found\n";
  }

  // Get sensors
  std::string imu_name = prefix + "head_imu_sensor";
  this->imuSensor =
    std::dynamic_pointer_cast<sensors::ImuSensor>
      (sensors::SensorManager::Instance()->GetSensor(imu_name));
  if (!this->imuSensor)
    gzerr << imu_name << " not found\n" << "\n";

  std::string motor_joint_name = prefix + "motor_joint";
  this->motor_joint = this->robotModel->GetJoint(motor_joint_name);
  if (!this->motor_joint)
  {
    gzerr << motor_joint_name << " not found, plugin will stop loading\n";
    return;
  }

  // publish joint states for spindle joint
  this->jointStates.name.resize(1);
  this->jointStates.position.resize(1);
  this->jointStates.velocity.resize(1);
  this->jointStates.effort.resize(1);

  // get fake joints and block them
  std::string camera_joint_name = prefix + "left_camera_optical_joint";
  this->camera_joint = this->robotModel->GetJoint(camera_joint_name);

  if (this->camera_joint) {
      // Setting limits to 0 so the joint doesn't move
      this->camera_joint->SetLowerLimit(0,0);
      this->camera_joint->SetUpperLimit(0,0);
  } else {
      gzwarn << camera_joint_name << " not found! Things may not work for ";
      gzwarn << "the simulated Hokuyo. Have you set the joint in ";
      gzwarn << "multisense_sl.gazebo.xacro file?" << std::endl;
  }

  std::string spindle_joint_name = prefix + "spindle_joint";
  this->spindle_joint = this->robotModel->GetJoint(spindle_joint_name);
  if(this->spindle_joint){
      // Setting limits to 0 so the joint doesn't move
      this->spindle_joint->SetLowerLimit(0,0);
      this->spindle_joint->SetUpperLimit(0,0);
  } else {
      gzwarn << spindle_joint_name << " not found! Things may not work for ";
      gzwarn << "the simulated Hokuyo. Have you set the joint in ";
      gzwarn << "multisense_sl.gazebo.xacro file?" << std::endl;
  }


  // sensors::Sensor_V s = sensors::SensorManager::Instance()->GetSensors();
  // for (sensors::Sensor_V::iterator siter = s.begin();
  //                                  siter != s.end(); ++siter)
  //   gzerr << (*siter)->GetName() << "\n";

  std::string stereo_name = prefix + "stereo_camera";
  this->multiCameraSensor =
    std::dynamic_pointer_cast<sensors::MultiCameraSensor>(
    sensors::SensorManager::Instance()->GetSensor(stereo_name));
  if (!this->multiCameraSensor)
    gzerr << stereo_name << " sensor not found\n";

  // get default frame rate
  this->multiCameraFrameRate = this->multiCameraSensor->UpdateRate();

  if (!sensors::SensorManager::Instance()->GetSensor("head_hokuyo_sensor"))
    gzerr << "laser sensor not found\n";

  if (!ros::isInitialized())
  {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
    return;
  }

  this->deferred_load_thread_ = boost::thread(
    boost::bind(&MultiSenseSL::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::LoadThread()
{
  // create ros node
  this->rosnode_ = new ros::NodeHandle("");

  // publish multi queue
  this->pmq->startServiceThread();

  this->rosNamespace = "/multisense";
  std::string robot_name = "/" + this->robotModel->GetName();

  // ros publications
  // publish joint states for tf (robot state publisher)
  this->pubJointStatesQueue = this->pmq->addPub<sensor_msgs::JointState>();
  this->pubJointStates = this->rosnode_->advertise<sensor_msgs::JointState>(
    robot_name + "/joint_states", 10);

  // publish imu data
  this->pubImuQueue = this->pmq->addPub<sensor_msgs::Imu>();
  this->pubImu =
    this->rosnode_->advertise<sensor_msgs::Imu>(
      this->rosNamespace + "/imu", 10);

  // ros subscription
  ros::SubscribeOptions set_spindle_speed_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->rosNamespace + "/set_spindle_speed", 100,
    boost::bind(static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetSpindleSpeed), this, _1),
    ros::VoidPtr(), &this->queue_);
  this->set_spindle_speed_sub_ =
    this->rosnode_->subscribe(set_spindle_speed_so);

  /// for deprecation from ~/multisense[_sl]/fps to ~/multisense[_sl]/set_fps
  /// per issue 272
  ros::SubscribeOptions set_multi_camera_frame_rate_so_old =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->rosNamespace + "/fps", 100,
    boost::bind(static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetMultiCameraFrameRateOld), this, _1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_frame_rate_sub_old_ =
    this->rosnode_->subscribe(set_multi_camera_frame_rate_so_old);

  ros::SubscribeOptions set_multi_camera_frame_rate_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->rosNamespace + "/set_fps", 100,
    boost::bind(static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetMultiCameraFrameRate), this, _1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_frame_rate_sub_ =
    this->rosnode_->subscribe(set_multi_camera_frame_rate_so);

  /* FIXME currently this causes simulation to crash,
  ros::SubscribeOptions set_multi_camera_resolution_so =
    ros::SubscribeOptions::create<std_msgs::Int32>(
    this->rosNamespace + "/set_camera_resolution_mode", 100,
    boost::bind(static_cast<void (MultiSenseSL::*)
      (const std_msgs::Int32::ConstPtr&)>(
        &MultiSenseSL::SetMultiCameraResolution), this, _1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_resolution_sub_ =
    this->rosnode_->subscribe(set_multi_camera_resolution_so);
  */

  /* not implemented, not supported
  ros::SubscribeOptions set_spindle_state_so =
    ros::SubscribeOptions::create<std_msgs::Bool>(
    this->rosNamespace + "/set_spindle_state", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Bool::ConstPtr&)>(
        &MultiSenseSL::SetSpindleState),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_spindle_state_sub_ =
    this->rosnode_->subscribe(set_spindle_state_so);

  ros::SubscribeOptions set_multi_camera_exposure_time_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->rosNamespace + "/set_camera_exposure_time", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetMultiCameraExposureTime),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_exposure_time_sub_ =
    this->rosnode_->subscribe(set_multi_camera_exposure_time_so);

  ros::SubscribeOptions set_multi_camera_gain_so =
    ros::SubscribeOptions::create<std_msgs::Float64>(
    this->rosNamespace + "/set_camera_gain", 100,
    boost::bind( static_cast<void (MultiSenseSL::*)
      (const std_msgs::Float64::ConstPtr&)>(
        &MultiSenseSL::SetMultiCameraGain),this,_1),
    ros::VoidPtr(), &this->queue_);
  this->set_multi_camera_gain_sub_ =
    this->rosnode_->subscribe(set_multi_camera_gain_so);
  */

  /// \todo: waiting for gen_srv to be implemented (issue #37)
  /* Advertise services on the custom queue
  std::string set_spindle_speed_service_name(
    this->rosNamespace + "/set_spindle_speed");
  ros::AdvertiseServiceOptions set_spindle_speed_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      set_spindle_speed_service_name,
      boost::bind(&MultiSenseSL::SetSpindleSpeed,this,_1,_2),
      ros::VoidPtr(), &this->queue_);
  this->set_spindle_speed_service_ =
    this->rosnode_->advertiseService(set_spindle_speed_aso);

  std::string set_spindle_state_service_name(
    this->rosNamespace + "/set_spindle_state");
  ros::AdvertiseServiceOptions set_spindle_state_aso =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      set_spindle_state_service_name,
      boost::bind(&MultiSenseSL::SetSpindleState,this,_1,_2),
      ros::VoidPtr(), &this->queue_);
  this->set_spindle_state_service_ =
    this->rosnode_->advertiseService(set_spindle_state_aso);
  */

  this->lastUpdateTime = this->world->SimTime().Double();
  this->updateRate = 1.0;

  // ros callback queue for processing subscription
  this->callback_queue_thread_ = boost::thread(
    boost::bind(&MultiSenseSL::QueueThread, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
     boost::bind(&MultiSenseSL::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::UpdateStates()
{
  common::Time curTime = this->world->SimTime();

  // get imu data from imu link
  if (this->imuSensor)
  {
    sensor_msgs::Imu imuMsg;
    imuMsg.header.frame_id = this->imuLinkName;
    imuMsg.header.stamp = ros::Time(curTime.Double());

    // compute angular rates
    {
      ignition::math::Vector3d wLocal = this->imuSensor->AngularVelocity();
      imuMsg.angular_velocity.x = wLocal.X();
      imuMsg.angular_velocity.y = wLocal.Y();
      imuMsg.angular_velocity.z = wLocal.Z();
    }

    // compute acceleration
    {
      ignition::math::Vector3d accel = this->imuSensor->LinearAcceleration();
      imuMsg.linear_acceleration.x = accel.X();
      imuMsg.linear_acceleration.y = accel.Y();
      imuMsg.linear_acceleration.z = accel.Z();
    }

    // compute orientation
    {
      ignition::math::Quaterniond imuRot =
        this->imuSensor->Orientation();
      imuMsg.orientation.x = imuRot.X();
      imuMsg.orientation.y = imuRot.Y();
      imuMsg.orientation.z = imuRot.Z();
      imuMsg.orientation.w = imuRot.W();
    }


    //
    // Publish the static transforms from our calibration.
    static_tf_broadcaster_.sendTransform(tf::StampedTransform(motor_to_camera_,
                                          ros::Time(curTime.Double()),left_camera_optical_,
                                          motor_));



    static_tf_broadcaster_.sendTransform(tf::StampedTransform(laser_to_spindle_,
                                          ros::Time(curTime.Double()), spindle_, hokuyo_));

    this->pubImuQueue->push(imuMsg, this->pubImu);
  }

  double dt = (curTime - this->lastTime).Double();
  if (dt > 0)
  {
    this->jointStates.header.stamp = ros::Time(curTime.sec, curTime.nsec);
    this->jointStates.name[0] = this->motor_joint->GetName();
    ignition::math::Angle angle = this->motor_joint->Position(0);
    angle.Normalize();
    this->jointStates.position[0] = angle.Radian();
    this->jointStates.velocity[0] = this->motor_joint->GetVelocity(0);
    this->jointStates.effort[0] = 0;

    if (this->spindleOn)
    {
      // PID control (velocity) spindle
      double spindleError = this->motor_joint->GetVelocity(0)
                          - this->spindleSpeed;
      double spindleCmd = this->spindlePID.Update(spindleError, dt);
      this->motor_joint->SetForce(0, spindleCmd);



      this->jointStates.effort[0] = spindleCmd;

      this->lastTime = curTime;
    }
    else
    {
      this->spindlePID.Reset();
    }
    this->pubJointStatesQueue->push(this->jointStates, this->pubJointStates);
  }
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
bool MultiSenseSL::SetSpindleSpeed(std_srvs::Empty::Request &/*req*/,
                                   std_srvs::Empty::Response &/*res*/)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool MultiSenseSL::SetSpindleState(std_srvs::Empty::Request &/*req*/,
                                   std_srvs::Empty::Response &/*res*/)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetSpindleSpeed(const std_msgs::Float64::ConstPtr &_msg)
{
  this->spindleSpeed = static_cast<double>(_msg->data);
  if (this->spindleSpeed > this->spindleMaxRPM * 2.0*M_PI / 60.0)
    this->spindleSpeed = this->spindleMaxRPM * 2.0*M_PI / 60.0;
  else if (this->spindleSpeed < this->spindleMinRPM * 2.0*M_PI / 60.0)
    this->spindleSpeed = this->spindleMinRPM * 2.0*M_PI / 60.0;
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetSpindleState(const std_msgs::Bool::ConstPtr &_msg)
{
  this->spindleOn = _msg->data;
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetMultiCameraFrameRateOld(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  ROS_WARN("Frame rate was modified but the ros topic ~/mutlisense_sl/fps"
           " has been replaced by ~/mutlisense_sl/set_fps per issue 272.");
  this->SetMultiCameraFrameRate(_msg);
}
////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetMultiCameraFrameRate(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  // limit frame rate to what is capable
  this->multiCameraFrameRate = static_cast<double>(_msg->data);

  // FIXME: Hardcoded lower limit on all resolution
  if (this->multiCameraFrameRate < 1.0)
  {
    ROS_INFO("Camera rate cannot be below 1Hz at any resolution\n");
    this->multiCameraFrameRate = 1.0;
  }

  // FIXME: Hardcoded upper limit.  Need to switch rates between modes.
  if (this->imagerMode == 0)
  {
    if (this->multiCameraFrameRate > 15.0)
    {
      ROS_INFO("Camera rate cannot be above 15Hz at this resolution\n");
      this->multiCameraFrameRate = 15.0;
    }
  }
  else if (this->imagerMode == 1)
  {
    if (this->multiCameraFrameRate > 30.0)
    {
      ROS_INFO("Camera rate cannot be above 30Hz at this resolution\n");
      this->multiCameraFrameRate = 30.0;
    }
  }
  else if (this->imagerMode == 2)
  {
    if (this->multiCameraFrameRate > 60.0)
    {
      ROS_INFO("Camera rate cannot be above 60Hz at this resolution\n");
      this->multiCameraFrameRate = 60.0;
    }
  }
  else if (this->imagerMode == 3)
  {
    if (this->multiCameraFrameRate > 70.0)
    {
      ROS_INFO("Camera rate cannot be above 70Hz at this resolution\n");
      this->multiCameraFrameRate = 70.0;
    }
  }
  else
  {
    ROS_ERROR("MultiSense SL internal state error (%d)", this->imagerMode);
  }

  this->multiCameraSensor->SetUpdateRate(this->multiCameraFrameRate);
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetMultiCameraResolution(
  const std_msgs::Int32::ConstPtr &_msg)
{
  /// see MultiSenseSLPlugin.h for available modes
  if (_msg->data < 0 || _msg->data > 3)
  {
    ROS_WARN("set_camera_resolution_mode must"
              " be between 0 - 3:\n"
              "  0 - 2MP (2048*1088) @ up to 15 fps\n"
              "  1 - 1MP (2048*544) @ up to 30 fps\n"
              "  2 - 0.5MP (1024*544) @ up to 60 fps (default)\n"
              "  3 - VGA (640*480) @ up to 70 fps\n");
    return;
  }

  this->imagerMode = _msg->data;

  unsigned int width = 640;
  unsigned int height = 480;
  if (this->imagerMode == 0)
  {
    width = 2048;
    height = 1088;
    if (this->multiCameraFrameRate > 15)
    {
      ROS_INFO("Reducing frame rate to 15Hz.");
      this->multiCameraFrameRate = 15.0;
    }
  }
  else if (this->imagerMode == 1)
  {
    width = 2048;
    height = 544;
    if (this->multiCameraFrameRate > 30)
    {
      ROS_INFO("Reducing frame rate to 30Hz.");
      this->multiCameraFrameRate = 30.0;
    }
  }
  else if (this->imagerMode == 2)
  {
    width = 1024;
    height = 544;
    if (this->multiCameraFrameRate > 60)
    {
      ROS_INFO("Reducing frame rate to 60Hz.");
      this->multiCameraFrameRate = 60.0;
    }
  }
  else if (this->imagerMode == 3)
  {
    width = 640;
    height = 480;
    if (this->multiCameraFrameRate > 70)
    {
      ROS_INFO("Reducing frame rate to 70Hz.");
      this->multiCameraFrameRate = 70.0;
    }
  }

  this->multiCameraSensor->SetUpdateRate(this->multiCameraFrameRate);

  for (unsigned int i = 0; i < this->multiCameraSensor->CameraCount(); ++i)
  {
    this->multiCameraSensor->Camera(i)->SetImageWidth(width);
    this->multiCameraSensor->Camera(i)->SetImageHeight(height);
  }
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetMultiCameraExposureTime(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  this->multiCameraExposureTime = static_cast<double>(_msg->data);
  gzwarn << "setting camera exposure time in sim not implemented\n";
}

////////////////////////////////////////////////////////////////////////////////
void MultiSenseSL::SetMultiCameraGain(const std_msgs::Float64::ConstPtr
                                          &_msg)
{
  this->multiCameraGain = static_cast<double>(_msg->data);
  gzwarn << "setting camera gain in sim not implemented\n";
}
}
