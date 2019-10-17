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
 * @file /xbot_node/src/node/xbot_node.cpp
 *
 * @brief Implementation for the ros xbot node wrapper.
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "xbot_node/xbot_ros.hpp"
#include <float.h>
#include <tf/tf.h>
#include <ecl/streams/string_stream.hpp>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace xbot {

/*****************************************************************************
 ** Implementation [XbotRos]
 *****************************************************************************/

/**
 * @brief Default constructor.
 *
 * Make sure you call the init() method to fully define this node.
 */
XbotRos::XbotRos(std::string& node_name)
    : name(node_name),
      cmd_vel_timed_out_(false),
      base_serial_timed_out_(false),
      sensor_serial_timed_out_(false),
      base_slot_stream_data(&XbotRos::processBaseStreamData, *this),
      sensor_slot_stream_data(&XbotRos::processSensorStreamData, *this) {}

/**
 * This will wait some time while xbot internally closes its threads and
 * destructs
 * itself.
 */
XbotRos::~XbotRos() {
  ROS_INFO_STREAM("Xbot : waiting for xbot thread to finish [" << name << "].");
  xbot.setSoundEnableControl(false);
  xbot.setLedControl(0);
}

bool XbotRos::init(ros::NodeHandle& nh) {
  /*********************
   ** Communications
   **********************/
  advertiseTopics(nh);
  subscribeTopics(nh);

  /*********************
   ** Slots
   **********************/
  base_slot_stream_data.connect(name + std::string("/base_stream_data"));
  sensor_slot_stream_data.connect(name + std::string("/sensor_stream_data"));
  /*********************
   ** Driver Parameters
   **********************/
  Parameters parameters;

  nh.param("acceleration_limiter", parameters.enable_acceleration_limiter,
           false);

  nh.param("led_indicate_battery", led_indicate_battery, true);

  parameters.sigslots_namespace =
      name;  // name is automatically picked up by device_nodelet parent.
  if (!nh.getParam("base_port", parameters.base_port)) {
    ROS_ERROR_STREAM(
        "Xbot : no base device port given on the parameter server (e.g. "
        "/dev/ttyUSB0)["
        << name << "].");
    return false;
  }

  if (!nh.getParam("sensor_port", parameters.sensor_port)) {
    ROS_ERROR_STREAM(
        "Xbot : no sensor device port given on the parameter server (e.g. "
        "/dev/ttyUSB0)["
        << name << "].");
    return false;
  }

  /*********************
   ** Joint States
   **********************/
  std::string robot_description, wheel_left_joint_name, wheel_right_joint_name;

  nh.param("wheel_left_joint_name", wheel_left_joint_name,
           std::string("wheel_left_joint"));
  nh.param("wheel_right_joint_name", wheel_right_joint_name,
           std::string("wheel_right_joint"));

  // minimalistic check: are joint names present on robot description file?
  if (!nh.getParam("/robot_description", robot_description)) {
    ROS_WARN("Xbot : no robot description given on the parameter server");
  } else {
    if (robot_description.find(wheel_left_joint_name) == std::string::npos) {
      ROS_WARN("Xbot : joint name %s not found on robot description",
               wheel_left_joint_name.c_str());
    }

    if (robot_description.find(wheel_right_joint_name) == std::string::npos) {
      ROS_WARN("Xbot : joint name %s not found on robot description",
               wheel_right_joint_name.c_str());
    }
  }
  //  joint_states.name.push_back(wheel_left_joint_name);
  //  joint_states.name.push_back(wheel_right_joint_name);
  //  joint_states.position.resize(2,0.0);
  //  joint_states.velocity.resize(2,0.0);
  //  joint_states.effort.resize(2,0.0);
  joint_states.name.resize(4);
  joint_states.position.resize(4);
  joint_states.velocity.resize(2);
  joint_states.name[0] = "left_wheel_hinge";
  joint_states.name[1] = "right_wheel_hinge";
  joint_states.name[2] = "base_to_yaw_platform";
  joint_states.name[3] = "yaw_to_pitch_platform";
  joint_states.position[0] = 0;
  joint_states.position[1] = 0;
  joint_states.position[2] = 0;
  joint_states.position[3] = 0;

  /*********************
   ** Validation
   **********************/
  if (!parameters.validate()) {
    ROS_ERROR_STREAM("Xbot : parameter configuration failed [" << name << "].");
    ROS_ERROR_STREAM("Xbot : " << parameters.error_msg << "[" << name << "]");
    return false;
  } else {
    if (parameters.simulation) {
      ROS_INFO("Xbot : driver going into loopback (simulation) mode.");
    } else {
      ROS_INFO_STREAM("Xbot : configured for connection on base_port "
                      << parameters.base_port << " [" << name << "].");
      ROS_INFO_STREAM("Xbot : configured for connection on sensor_port "
                      << parameters.sensor_port << " [" << name << "].");
      ROS_INFO_STREAM("Xbot : driver running in normal (non-simulation) mode"
                      << " [" << name << "].");
    }
  }

  odometry.init(nh, name);

  /*********************
   ** Driver Init
   **********************/
  try {
    xbot.init(parameters);

    //    xbot.setBaseControl(0.4,0);

    ros::Duration(0.1).sleep();  // wait for some data to come in.
    if (!xbot.base_isAlive()) {
      ROS_WARN_STREAM(
          "Xbot : no base data stream, is base board connected or turned on?");
      // don't need to return false here - simply turning xbot on while
      // spin()'ing should resurrect the situation.
    }
    if (!xbot.sensor_isAlive()) {
      ROS_WARN_STREAM(
          "Xbot : no sensor data stream, is sensor board connected or turned "
          "on?");
      // don't need to return false here - simply turning xbot on while
      // spin()'ing should resurrect the situation.
    }
    xbot.enable();
  } catch (const ecl::StandardException& e) {
    switch (e.flag()) {
      case (ecl::OpenError): {
        ROS_ERROR_STREAM("Xbot : could not open connection ["
                         << parameters.base_port << "][" << name << "].");
        ROS_ERROR_STREAM("Xbot : could not open connection ["
                         << parameters.sensor_port << "][" << name << "].");
        break;
      }
      default: {
        ROS_ERROR_STREAM("Xbot : initialisation failed [" << name << "].");
        ROS_DEBUG_STREAM(e.what());
        break;
      }
    }
    return false;
  }
  // xbot.printSigSlotConnections();
  xbot.setSoundEnableControl(true);
  return true;
}
/**
 * This is a worker function that runs in a background thread initiated by
 * the nodelet. It gathers diagnostics information from the xbot driver,
 * and broadcasts the results to the rest of the ros ecosystem.
 *
 * Note that the actual driver data is collected via the slot callbacks in this
 * class.
 *
 * @return Bool : true/false if successfully updated or not (xbot driver
 * shutdown).
 */
bool XbotRos::update() {
  if (xbot.isShutdown()) {
    ROS_ERROR_STREAM("Xbot : Driver has been shutdown. Stopping update loop. ["
                     << name << "].");
    return false;
  }

  if ((xbot.isEnabled() == true) && odometry.commandTimeout()) {
    if (!cmd_vel_timed_out_) {
      xbot.setBaseControl(0, 0);
      cmd_vel_timed_out_ = true;
      //      ROS_ERROR("Xbot : Incoming velocity commands not received for more
      //      than %.2f seconds -> zero'ing velocity commands",
      //      odometry.timeout().toSec());
    }
  } else {
    cmd_vel_timed_out_ = false;
  }

  bool base_is_alive = xbot.base_isAlive();
  if (!base_is_alive) {
    if (!base_serial_timed_out_) {
      ROS_ERROR_STREAM(
          "Xbot : Timed out while waiting for base serial data stream ["
          << name << "].");
      base_serial_timed_out_ = true;
    } else {
      base_serial_timed_out_ = false;
    }
  }

  bool sensor_is_alive = xbot.sensor_isAlive();
  if (!sensor_is_alive) {
    if (!sensor_serial_timed_out_) {
      ROS_ERROR_STREAM(
          "Xbot : Timed out while waiting for sensor serial data stream ["
          << name << "].");
      sensor_serial_timed_out_ = true;
    } else {
      sensor_serial_timed_out_ = false;
    }
  }

  return true;
}

/**
 * Two groups of publishers, one required by turtlebot, the other for
 * xbot esoterics.
 */
void XbotRos::advertiseTopics(ros::NodeHandle& nh) {
  /*********************
  ** Joint state publisher init
  **********************/
  joint_state_publisher =
      nh.advertise<sensor_msgs::JointState>("joint_states", 100);

  /*********************
  ** Xbot publisher init
  **********************/
  core_sensor_publisher =
      nh.advertise<xbot_msgs::CoreSensor>("sensors/core", 100);
  extra_sensor_publisher =
      nh.advertise<xbot_msgs::ExtraSensor>("sensors/extra", 100);
  yaw_platform_state_publisher =
      nh.advertise<std_msgs::Int8>("sensors/yaw_platform_degree", 100);
  pitch_platform_state_publisher =
      nh.advertise<std_msgs::Int8>("sensors/pitch_platform_degree", 100);
  stop_buttom_state_publisher =
      nh.advertise<std_msgs::Bool>("sensors/motor_disabled", 100);
  sound_state_publisher =
      nh.advertise<std_msgs::Bool>("sensors/sound_enabled", 100);
  battery_state_publisher =
      nh.advertise<xbot_msgs::Battery>("sensors/battery", 100);

  echo_data_publisher = nh.advertise<xbot_msgs::Echo>("sensors/echo", 100);
  infrared_data_publisher =
      nh.advertise<xbot_msgs::InfraRed>("sensors/infrared", 100);

  imu_data_publisher = nh.advertise<sensor_msgs::Imu>("/imu", 100);
  raw_imu_data_publisher =
      nh.advertise<xbot_msgs::RawImu>("sensors/raw_imu_data", 100);
  robot_state_publisher = nh.advertise<xbot_msgs::XbotState>("xbot/state", 100);
}

/**
 * Two groups of subscribers, one required by turtlebot, the other for
 * xbot esoterics.
 */
void XbotRos::subscribeTopics(ros::NodeHandle& nh) {
  motor_enable_command_subscriber =
      nh.subscribe(std::string("commands/motor_disable"), 10,
                   &XbotRos::subscribeMotorEnableCommand, this);
  velocity_command_subscriber =
      nh.subscribe(std::string("commands/velocity"), 10,
                   &XbotRos::subscribeVelocityCommand, this);
  yaw_platform_command_subscriber =
      nh.subscribe(std::string("commands/yaw_platform"), 10,
                   &XbotRos::subscribeYawPlatformCommand, this);
  pitch_platform_command_subscriber =
      nh.subscribe(std::string("commands/pitch_platform"), 10,
                   &XbotRos::subscribePitchPlatformCommand, this);
  sound_command_subscriber =
      nh.subscribe(std::string("commands/sound_enable"), 10,
                   &XbotRos::subscribeSoundCommand, this);
  led_command_subscriber = nh.subscribe(std::string("commands/led"), 10,
                                        &XbotRos::subscribeLedCommand, this);
  lift_command_subscirber =
      nh.subscribe("commands/lift", 10, &XbotRos::subscribeLiftCommand, this);
  reset_odometry_subscriber = nh.subscribe(
      "commands/reset_odometry", 10, &XbotRos::subscribeResetOdometry, this);
}

}  // namespace xbot
