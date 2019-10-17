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
 * @file src/node/slot_callbacks.cpp
 *
 * @brief All the slot callbacks for interrupts from the xbot driver.
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <math.h>
#include "xbot_node/xbot_ros.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace xbot {

void XbotRos::processBaseStreamData() {
  publishWheelState();
  publishCoreSensor();
  publishEchoData();
  publishInfraredData();
  publishStopButtonState();
  publishBatteryState();
  publishRobotState();
}

/*****************************************************************************
** Publish Sensor Stream Workers
*****************************************************************************/
void XbotRos::publishWheelState() {
  //     Take latest encoders and gyro data
  ecl::Pose2D<double> pose_update;
  ecl::linear_algebra::Vector3d pose_update_rates;
  xbot.updateOdometry(pose_update, pose_update_rates);
  float left_joint_pos, left_joint_vel, right_joint_pos, right_joint_vel;
  xbot.getWheelJointStates(left_joint_pos, left_joint_vel, right_joint_pos,
                           right_joint_vel);  // right wheel
  joint_states.position[0] = (double)left_joint_pos;
  joint_states.velocity[0] = left_joint_vel;
  joint_states.position[1] = (double)right_joint_pos;
  joint_states.velocity[1] = right_joint_vel;
  //    // Update and publish odometry and joint states
  //        ROS_ERROR_STREAM("imu_heading:" << xbot.getHeading());
  odometry.update(pose_update, pose_update_rates, xbot.getHeading(),
                  xbot.getAngularVelocity());

  if (ros::ok()) {
    joint_states.header.stamp = ros::Time::now();
    joint_state_publisher.publish(joint_states);
  }
}

void XbotRos::publishCoreSensor() {
  if (ros::ok()) {
    if (core_sensor_publisher.getNumSubscribers() > 0) {
      xbot_msgs::CoreSensor core_sensor;
      CoreSensors::Data data = xbot.getCoreSensorData();

      core_sensor.header.stamp = ros::Time::now();
      core_sensor.left_encoder = data.left_encoder;
      core_sensor.right_encoder = data.right_encoder;
      core_sensor.battery_percent = data.power_percent;
      core_sensor.ischarging = data.is_charging;
      core_sensor.front_echo = data.front_echo;
      core_sensor.rear_echo = data.rear_echo;
      core_sensor.front_infrared = data.front_infrared;
      core_sensor.rear_infrared = data.rear_infrared;
      core_sensor.error_state = data.error_state;
      core_sensor.left_motor_current = data.left_motor_current;
      core_sensor.right_motor_current = data.right_motor_current;
      core_sensor.motor_disabled = data.stop_button_state;
      core_sensor.time_stamp = data.timestamp;
      core_sensor.version = data.version;

      core_sensor_publisher.publish(core_sensor);
    }
  }
}

void XbotRos::publishEchoData() {
  if (ros::ok()) {
    if (echo_data_publisher.getNumSubscribers() > 0) {
      xbot_msgs::Echo msg;
      CoreSensors::Data data_echo = xbot.getCoreSensorData();
      msg.header.frame_id = "echo_link";
      msg.header.stamp = ros::Time::now();
      msg.front = data_echo.front_echo;
      msg.rear = data_echo.rear_echo;
      msg.front_near = (data_echo.front_echo < 1600);
      msg.rear_near = (data_echo.rear_echo < 1600);
      echo_data_publisher.publish(msg);
    }
  }
}

void XbotRos::publishInfraredData() {
  if (ros::ok()) {
    if (infrared_data_publisher.getNumSubscribers() > 0) {
      xbot_msgs::InfraRed msg;
      CoreSensors::Data data_core = xbot.getCoreSensorData();
      msg.header.frame_id = "infrared_link";
      msg.header.stamp = ros::Time::now();
      msg.front = data_core.front_infrared;
      msg.rear = data_core.rear_infrared;
      msg.front_hanged = (data_core.front_infrared > 2000);
      msg.rear_hanged = (data_core.rear_infrared > 2000);
      infrared_data_publisher.publish(msg);
    }
  }
}

void XbotRos::publishStopButtonState() {
  if (ros::ok()) {
    if (stop_buttom_state_publisher.getNumSubscribers() > 0) {
      std_msgs::Bool msg;
      CoreSensors::Data data_core = xbot.getCoreSensorData();
      msg.data = data_core.stop_button_state;
      stop_buttom_state_publisher.publish(msg);
    }
  }
}

void XbotRos::publishBatteryState() {
  if (ros::ok()) {
    //    if(battery_state_publisher.getNumSubscribers()>0)
    //    {
    xbot_msgs::Battery msg;
    CoreSensors::Data data_core = xbot.getCoreSensorData();
    msg.header.stamp = ros::Time::now();
    msg.is_charging = data_core.is_charging;

    msg.battery_percent = data_core.power_percent;
    battery_state_publisher.publish(msg);

    if (led_indicate_battery) {
      unsigned char leds = msg.battery_percent / 25 + 1;
      leds = pow(2, leds) - 1;
      xbot.setLedControl(leds);
    }

    //    }
  }
}

void XbotRos::publishRobotState() {
  //    ros::Rate r(50);
  if (ros::ok() && (robot_state_publisher.getNumSubscribers() > 0)) {
    xbot_msgs::XbotState msg;
    CoreSensors::Data core_data = xbot.getCoreSensorData();
    Sensors::Data extra_data = xbot.getExtraSensorsData();
    msg.header.stamp = ros::Time::now();
    msg.base_is_connected = xbot.is_base_connected();
    msg.sensor_is_connected = xbot.is_sensor_connected();
    msg.echo_plug_error = core_data.error_state;
    msg.infrared_plug_error = core_data.error_state;
    msg.motor_error = core_data.error_state;
    msg.version = core_data.version;

    robot_state_publisher.publish(msg);
    //        r.sleep();
  }
}

void XbotRos::processSensorStreamData() {
  publishExtraSensor();
  publishInertia();
  publishRawInertia();
  publishPitchPlatformState();
  publishYawPlatformState();
  publishSoundState();
}
void XbotRos::publishExtraSensor() {
  if (ros::ok()) {
    if (extra_sensor_publisher.getNumSubscribers() > 0) {
      xbot_msgs::ExtraSensor extra_sensor;

      Sensors::Data data = xbot.getExtraSensorsData();

      extra_sensor.header.stamp = ros::Time::now();

      extra_sensor.yaw_platform_degree = data.yaw_platform_degree;
      extra_sensor.pitch_platform_degree = data.pitch_platform_degree;
      extra_sensor.sound_is_mutex = data.sound_status;
      extra_sensor.acc_x = data.acc_x;
      extra_sensor.acc_y = data.acc_y;
      extra_sensor.acc_z = data.acc_z;
      extra_sensor.gyro_x = data.gyro_x;
      extra_sensor.gyro_y = data.gyro_y;
      extra_sensor.gyro_z = data.gyro_z;
      extra_sensor.mag_x = data.mag_x;
      extra_sensor.mag_y = data.mag_y;
      extra_sensor.mag_z = data.mag_z;
      extra_sensor.yaw = data.yaw;
      extra_sensor.pitch = data.pitch;
      extra_sensor.roll = data.roll;
      extra_sensor.q1 = data.q1;
      extra_sensor.q2 = data.q2;
      extra_sensor.q3 = data.q3;
      extra_sensor.q4 = data.q4;
      extra_sensor.error_state = data.error_status;
      extra_sensor.time_stamp = data.timestamp;
      extra_sensor.version = data.version;
      extra_sensor_publisher.publish(extra_sensor);
    }
  }
}

void XbotRos::publishInertia() {
  if (ros::ok()) {
    sensor_msgs::Imu imu_msg;

    Sensors::Data data = xbot.getExtraSensorsData();
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation = tf::createQuaternionMsgFromYaw(xbot.getHeading());
    imu_msg.orientation_covariance[0] = 10.01;
    imu_msg.orientation_covariance[4] = 10.01;
    imu_msg.orientation_covariance[8] = 10.01;

    imu_msg.angular_velocity.x = data.gyro_x;
    imu_msg.angular_velocity.y = data.gyro_y;
    imu_msg.angular_velocity.z = data.gyro_z;
    imu_msg.angular_velocity_covariance[0] = 10.01;
    imu_msg.angular_velocity_covariance[4] = 10.01;
    imu_msg.angular_velocity_covariance[8] = 10.01;

    imu_msg.linear_acceleration.x = data.acc_x;
    imu_msg.linear_acceleration.y = data.acc_y;
    imu_msg.linear_acceleration.z = data.acc_z;
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;

    imu_data_publisher.publish(imu_msg);
  }
}

void XbotRos::publishRawInertia() {
  if (ros::ok()) {
    if (raw_imu_data_publisher.getNumSubscribers() > 0) {
      xbot_msgs::RawImu raw_imu_msg;

      Sensors::Data data = xbot.getExtraSensorsData();

      raw_imu_msg.header.stamp = ros::Time::now();
      raw_imu_msg.acc_x = data.acc_x;
      raw_imu_msg.acc_y = data.acc_y;
      raw_imu_msg.acc_z = data.acc_z;
      raw_imu_msg.gyro_x = data.gyro_x;
      raw_imu_msg.gyro_y = data.gyro_y;
      raw_imu_msg.gyro_z = data.gyro_z;
      raw_imu_msg.mag_x = data.mag_x;
      raw_imu_msg.mag_y = data.mag_y;
      raw_imu_msg.mag_z = data.mag_z;
      raw_imu_data_publisher.publish(raw_imu_msg);
    }
  }
}

void XbotRos::publishYawPlatformState() {
  if (ros::ok()) {
    if (yaw_platform_state_publisher.getNumSubscribers() > 0) {
      std_msgs::Int8 yaw_platform_degree;
      Sensors::Data data = xbot.getExtraSensorsData();
      yaw_platform_degree.data = data.yaw_platform_degree - 120;

      yaw_platform_state_publisher.publish(yaw_platform_degree);
    }
  }
}

void XbotRos::publishPitchPlatformState() {
  if (ros::ok()) {
    if (pitch_platform_state_publisher.getNumSubscribers() > 0) {
      std_msgs::Int8 pitch_platform_degree;
      Sensors::Data data = xbot.getExtraSensorsData();
      pitch_platform_degree.data = data.pitch_platform_degree - 120;

      pitch_platform_state_publisher.publish(pitch_platform_degree);
    }
  }
}

void XbotRos::publishSoundState() {
  if (ros::ok()) {
    if (sound_state_publisher.getNumSubscribers() > 0) {
      std_msgs::Bool sound_is_mute;
      Sensors::Data data = xbot.getExtraSensorsData();
      sound_is_mute.data = data.sound_status;

      sound_state_publisher.publish(sound_is_mute);
    }
  }
}

}  // namespace xbot
