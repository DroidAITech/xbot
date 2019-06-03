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

#include "xbot_node/xbot_ros.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace xbot
{

void XbotRos::processStreamData() {
  publishWheelState();
  publishSensorState();
  publishDockIRData();
  publishEchoData();
  publishInfraredData();
  publishInertia();
  publishRawInertia();
  publishDebugSensors();
  publishRobotState();

}

/*****************************************************************************
** Publish Sensor Stream Workers
*****************************************************************************/

void XbotRos::publishSensorState()
{
  if ( ros::ok() ) {
//      float heading = xbot.getHeading();

//      ROS_ERROR("%f",heading);
    if (sensor_state_publisher.getNumSubscribers() > 0) {
      xbot_msgs::SensorState state;
      CoreSensors::Data data = xbot.getCoreSensorData();
      state.header.stamp = ros::Time::now();
      state.battery_voltage = data.battery_voltage;
      state.front_left_encoder = data.front_left_encoder;
      state.front_right_encoder = data.front_right_encoder;
      state.rear_left_encoder = data.rear_left_encoder;
      state.rear_right_encoder = data.rear_right_encoder;
      state.up_down_encoder = data.up_down_encoder;
      state.up_down_current = data.up_down_current;
      state.front_left_current = data.front_left_current;
      state.front_right_current = data.front_right_current;
      state.rear_left_current = data.rear_left_current;
      state.rear_right_current = data.rear_right_current;

      sensor_state_publisher.publish(state);
    }
  }
}

void XbotRos::publishWheelState()
{

//     Take latest encoders and gyro data
    ecl::Pose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    xbot.updateOdometry(pose_update, pose_update_rates);
    float left_joint_pos,left_joint_vel,right_joint_pos,right_joint_vel;
    xbot.getWheelJointStates(left_joint_pos,left_joint_vel,right_joint_pos,right_joint_vel);  // right wheel
    joint_states.position[0]=(double)left_joint_pos;
    joint_states.velocity[0]=left_joint_vel;
    joint_states.position[1]=(double)right_joint_pos;
//    ROS_ERROR_STREAM("jointstates.position[0]:" << right_joint_pos);

    joint_states.velocity[1]=right_joint_vel;
//    // Update and publish odometry and joint states
//        ROS_ERROR_STREAM("imu_heading:" << xbot.getHeading());
    odometry.update(pose_update, pose_update_rates, xbot.getHeading(), xbot.getAngularVelocity());

    if (ros::ok())
    {
      joint_states.header.stamp = ros::Time::now();
      joint_state_publisher.publish(joint_states);

    }
}

void XbotRos::publishInertia()
{
  if (ros::ok())
  {
    if (imu_data_publisher.getNumSubscribers() > 0)
    {
      // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
      sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);

      msg->header.frame_id = "gyro_link";
      msg->header.stamp = ros::Time::now();

      msg->orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, xbot.getHeading());


      // set a non-zero covariance on unused dimensions (pitch and roll); this is a requirement of robot_pose_ekf
      // set yaw covariance as very low, to make it dominate over the odometry heading when combined
      // 1: fill once, as its always the same;  2: using an invented value; cannot we get a realistic estimation?
      msg->orientation_covariance[0] = DBL_MAX;
      msg->orientation_covariance[4] = DBL_MAX;
      msg->orientation_covariance[8] = 0.05;

      // fill angular velocity; we ignore acceleration for now
      msg->angular_velocity.z = xbot.getAngularVelocity();

      // angular velocity covariance; useless by now, but robot_pose_ekf's
      // roadmap claims that it will compute velocities in the future
      msg->angular_velocity_covariance[0] = DBL_MAX;
      msg->angular_velocity_covariance[4] = DBL_MAX;
      msg->angular_velocity_covariance[8] = 0.05;

      imu_data_publisher.publish(msg);
    }
  }
}

void XbotRos::publishRawInertia()
{
  if ( ros::ok() && (raw_imu_data_publisher.getNumSubscribers() > 0) )
  {
    // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
    xbot_msgs::ImuNinePtr msg(new xbot_msgs::ImuNine);
    msg->accex = xbot.getCoreSensorData().acce_x;
    msg->gyrox = xbot.getCoreSensorData().gyro_x;
    msg->magx = xbot.getCoreSensorData().mag_x;
    msg->accey = xbot.getCoreSensorData().acce_y;
    msg->gyroy = xbot.getCoreSensorData().gyro_y;
    msg->magy = xbot.getCoreSensorData().mag_y;
    msg->accez = xbot.getCoreSensorData().acce_z;
    msg->gyroz = xbot.getCoreSensorData().gyro_z;
    msg->magz = xbot.getCoreSensorData().mag_z;

    raw_imu_data_publisher.publish(msg);
  }
}

void XbotRos::publishDebugSensors()
{
    ros::Rate r(50);
    if ( ros::ok() && (debug_sensors_publisher.getNumSubscribers() > 0) )
    {
        xbot_msgs::DebugSensorPtr msg(new xbot_msgs::DebugSensor);
        CoreSensors::Data data_debug = xbot.getCoreSensorData();

        msg->header.frame_id = "encoder";
        msg->header.stamp = ros::Time::now();
        msg->data.push_back(data_debug.front_left_encoder);
        msg->data.push_back(data_debug.front_right_encoder);
        msg->heading=xbot.getHeading();
        debug_sensors_publisher.publish(msg);
        r.sleep();

    }

}

void XbotRos::publishRobotState()
{
//    ros::Rate r(50);
    if ( ros::ok() && (robot_state_publisher.getNumSubscribers() > 0) )
    {
        xbot_msgs::XbotStatePtr msg(new xbot_msgs::XbotState);
        CoreSensors::Data data = xbot.getCoreSensorData();
        msg->header.stamp = ros::Time::now();
        msg->power_percent = (data.battery_voltage - xbot_msgs::XbotState::MIN_VOLTAGE)*100/(xbot_msgs::XbotState::MAX_VOLTAGE - xbot_msgs::XbotState::MIN_VOLTAGE);
        msg->height_percent = xbot.getHeightPercent();
        msg->cloud_degree = xbot.getPlatformDegree();
        msg->camera_degree = xbot.getCameraDegree();
        msg->dock_left_infred = data.dock_left_infred;
        msg->dock_right_infred = data.dock_right_infred;
        msg->front_left_hanged = (data.front_left_infred<1000);
        msg->front_center_hanged = (data.front_center_infred<1000);
        msg->front_right_hanged = (data.front_right_infred<1000);
        msg->rear_left_hanged = (data.rear_left_infred<1000);
        msg->rear_center_hanged = (data.rear_center_infred<1000);
        msg->rear_right_hanged = (data.rear_right_infred<1000);
        msg->is_hanged = msg->front_left_hanged&&msg->front_center_hanged&&msg->front_right_hanged&&msg->rear_left_hanged&&msg->rear_center_hanged&&msg->rear_right_hanged;
        msg->front_left_near = (data.front_left_echo<1300);
        msg->front_center_near = (data.front_center_echo<1300);
        msg->front_right_near = (data.front_right_echo<1300);
        msg->rear_left_near = (data.rear_left_echo<1300);
        msg->rear_center_near = (data.rear_center_echo<1300);
        msg->rear_right_near = (data.rear_right_echo<1300);
        msg->is_obstacle = msg->front_left_near&&msg->front_center_near&&msg->front_right_near&&msg->rear_left_near&&msg->rear_center_near&&msg->rear_right_near;

        robot_state_publisher.publish(msg);
//        r.sleep();

    }

}

void XbotRos::publishDockIRData()
{
    if (ros::ok())
    {
        if (dock_ir_publisher.getNumSubscribers() > 0)
        {

            // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
            xbot_msgs::DockInfraRedPtr msg(new xbot_msgs::DockInfraRed);
            CoreSensors::Data data_infred = xbot.getCoreSensorData();
            msg->header.frame_id = "dock_ir_link";
            msg->header.stamp = ros::Time::now();
            msg->rear_left_infred=(data_infred.rear_left_infred<=0.07)?1:8;
            msg->rear_center_infred=(data_infred.rear_center_infred<=0.2)?2:16;
            msg->rear_right_infred=(data_infred.rear_right_infred<=0.07)?4:32;
            dock_ir_publisher.publish(msg);
        }
    }
}

void XbotRos::publishEchoData()
{
  if(ros::ok())
  {
    if(echo_data_publisher.getNumSubscribers()>0)
    {
      xbot_msgs::Echos msg;
      CoreSensors::Data data_echo = xbot.getCoreSensorData();
      msg.header.frame_id = "echo_link";
      msg.header.stamp = ros::Time::now();
      msg.front_left = data_echo.front_left_echo;
      msg.front_center = data_echo.front_center_echo;
      msg.front_right = data_echo.front_right_echo;
      msg.rear_left = data_echo.rear_left_echo;
      msg.rear_center = data_echo.rear_center_echo;
      msg.rear_right = data_echo.rear_right_echo;
      msg.front_left_near = (data_echo.front_left_echo<1300);
      msg.front_center_near = (data_echo.front_center_echo<1300);
      msg.front_right_near = (data_echo.front_right_echo<1300);
      msg.rear_left_near = (data_echo.rear_left_echo<1300);
      msg.rear_center_near = (data_echo.rear_center_echo<1300);
      msg.rear_right_near = (data_echo.rear_right_echo<1300);


//      int near_left = (data_echo.front_left_echo<=0.07)?1:0;
//      int near_center = (data_echo.front_center_echo<=0.2)?2:0;
//      int near_right = (data_echo.front_right_echo<=0.07)?4:0;
//      msg->near = near_left+near_center+near_right;
      echo_data_publisher.publish(msg);

    }
  }



}
void XbotRos::publishInfraredData()
{
  if(ros::ok())
  {
    if(infrared_data_publisher.getNumSubscribers()>0)
    {
      xbot_msgs::InfraRed msg;
      CoreSensors::Data data_core = xbot.getCoreSensorData();
      msg.header.frame_id = "infrared_link";
      msg.header.stamp = ros::Time::now();
      msg.front_left = data_core.front_left_infred;
      msg.front_center = data_core.front_center_infred;
      msg.front_right = data_core.front_right_infred;
      msg.rear_left = data_core.rear_left_infred;
      msg.rear_center = data_core.rear_center_infred;
      msg.rear_right = data_core.rear_right_infred;
      msg.front_left_hanged = (data_core.front_left_infred<1000);
      msg.front_center_hanged = (data_core.front_center_infred<1000);
      msg.front_right_hanged = (data_core.front_right_infred<1000);
      msg.rear_left_hanged = (data_core.rear_left_infred<1000);
      msg.rear_center_hanged = (data_core.rear_center_infred<1000);
      msg.rear_right_hanged = (data_core.rear_right_infred<1000);


      infrared_data_publisher.publish(msg);
    }
  }
}

} // namespace xbot
