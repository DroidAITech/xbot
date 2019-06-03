/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file /xbot_node/include/xbot_node/xbot_ros.hpp
 *
 * @brief Wraps the xbot driver in a ROS-specific library
 *
 **/

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef XBOT_ROS_HPP_
#define XBOT_ROS_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <angles/angles.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <ecl/sigslots.hpp>
#include <xbot_msgs/DockInfraRed.h>
#include <xbot_msgs/SensorState.h>
#include <xbot_msgs/DebugSensor.h>
#include <xbot_msgs/Echos.h>
#include <xbot_msgs/InfraRed.h>
#include <xbot_driver/xbot.hpp>
#include <xbot_msgs/CloudCamera.h>
#include <xbot_msgs/Lift.h>
#include <xbot_msgs/XbotState.h>
#include <xbot_msgs/ImuNine.h>
#include <xbot_msgs/Power.h>
#include <xbot_msgs/ImuNine.h>
#include "odometry.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace xbot
{
class XbotRos
{
public:
  XbotRos(std::string& node_name);
  ~XbotRos();
  bool init(ros::NodeHandle& nh);
  bool update();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  /*********************
   ** Variables
   **********************/
  std::string name; // name of the ROS node
  Xbot xbot;
  sensor_msgs::JointState joint_states;
  Odometry odometry;
  bool cmd_vel_timed_out_; // stops warning spam when cmd_vel flags as timed out more than once in a row
  bool serial_timed_out_; // stops warning spam when serial connection timed out more than once in a row

  /*********************
   ** Ros Comms
   **********************/
  ros::Publisher imu_data_publisher;
  ros::Publisher raw_imu_data_publisher;
  ros::Publisher sensor_state_publisher;
  ros::Publisher joint_state_publisher;
  ros::Publisher dock_ir_publisher;
  ros::Publisher infrared_data_publisher;
  ros::Publisher echo_data_publisher;
  ros::Publisher raw_control_command_publisher;

  ros::Publisher debug_sensors_publisher;
  ros::Publisher robot_state_publisher;

  ros::Subscriber velocity_command_subscriber;
  ros::Subscriber lift_command_subscirber;
  ros::Subscriber cloudplatform_command_subscriber;
  ros::Subscriber power_command_subscriber;
  ros::Subscriber reset_odometry_subscriber;
  ros::Subscriber cloud_camera_subscriber;

  void advertiseTopics(ros::NodeHandle& nh);
  void subscribeTopics(ros::NodeHandle& nh);

  /*********************
  ** Ros Callbacks
  **********************/
  void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr);
  void subscribeLiftCommand(const xbot_msgs::LiftConstPtr);
  void subscribeCloudCameraCommand(const xbot_msgs::CloudCameraConstPtr);
  void subscribeResetOdometry(const std_msgs::EmptyConstPtr);
  void subscribePowerCommand(const xbot_msgs::PowerConstPtr);

  /*********************
   ** SigSlots
   **********************/
  ecl::Slot<> slot_stream_data;

  /*********************
   ** Slot Callbacks
   **********************/
  void publishWheelState();
  void processStreamData();
  void publishInertia();
  void publishRawInertia();
  void publishSensorState();
  void publishDockIRData();
  void publishEchoData();
  void publishInfraredData();
  void publishDebugSensors();
  void publishRobotState();



};

} // namespace xbot

#endif /* XBOT_ROS_HPP_ */
