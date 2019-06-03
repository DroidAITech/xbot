/**
 * @file /xbot_driver/src/driver/diff_drive.cpp
 *
 * @brief Differential drive abstraction (brought in from ycs).
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/xbot_driver/modules/diff_drive.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Implementation
*****************************************************************************/
DiffDrive::DiffDrive() :
  last_velocity_left(0.0),
  last_velocity_right(0.0),
  last_tick_left(0),
  last_tick_right(0),
  last_rad_left(0.0),
  last_rad_right(0.0),
//  v(0.0), w(0.0), // command velocities, in [m/s] and [rad/s]
  angular_velocity(0.0), linear_velocity(0.0), // command velocities, in [mm] and [mm/s]
  point_velocity(2,0.0), // command velocities, in [m/s] and [rad/s]
  bias(0.45), // wheelbase, wheel_to_wheel, in [m]
  wheel_radius(0.098), // radius of main wheel, in [m]
  tick_to_rad(0.0003925740f*1.6/2),//xbot1 8000tick/loop, xbot 32000tick/loop
  diff_drive_kinematics(bias, wheel_radius)
{}

/**
 * @brief Updates the odometry from firmware stamps and encoders.
 *
 * Really horrible - could do with an overhaul.
 *
 * @param time_stamp
 * @param left_encoder
 * @param right_encoder
 * @param pose_update
 * @param pose_update_rates
 */
void DiffDrive::update(const unsigned int &time_stamp,
                       const uint16_t &left_encoder,
                       const uint16_t &right_encoder,
                       ecl::Pose2D<double> &pose_update,
                       ecl::linear_algebra::Vector3d &pose_update_rates) {
  state_mutex.lock();
  static bool init_l = false;
  static bool init_r = false;
  float left_diff_ticks = 0.0f;
  float right_diff_ticks = 0.0f;
  unsigned short curr_tick_left = 0;
  unsigned short curr_tick_right = 0;
//  unsigned short tmp_diff_time = 0;
  unsigned int curr_timestamp = 0;
  curr_timestamp = time_stamp;
  curr_tick_left = left_encoder;
  if (!init_l)
  {
    last_tick_left = curr_tick_left;
    init_l = true;
  }
  left_diff_ticks = (float)(short)((curr_tick_left - last_tick_left) & 0xffff);
  last_tick_left = curr_tick_left;
  last_rad_left -= tick_to_rad * left_diff_ticks;

  curr_tick_right = right_encoder;
  if (!init_r)
  {
    last_tick_right = curr_tick_right;
    init_r = true;
  }
  right_diff_ticks = (float)(short)((curr_tick_right - last_tick_right) & 0xffff);
  last_tick_right = curr_tick_right;
  last_rad_right -= tick_to_rad * right_diff_ticks;

  // TODO this line and the last statements are really ugly; refactor, put in another place
  pose_update = diff_drive_kinematics.forward((double)(tick_to_rad * left_diff_ticks), (double)(tick_to_rad * right_diff_ticks));
//  std::cout<<"1"<<pose_update<<std::endl;

  if (curr_timestamp != last_timestamp)
  {

    last_diff_time =((double)(short)((curr_timestamp - last_timestamp) & 0xffff )) / 1000000.0f;
    std::cout<<"diff_time: "<<last_diff_time<<std::endl;
    last_timestamp = curr_timestamp;
    last_velocity_left = (tick_to_rad * left_diff_ticks) / last_diff_time;
    last_velocity_right = (tick_to_rad * right_diff_ticks) / last_diff_time;
  } else {
    // we need to set the last_velocity_xxx to zero?
  }

  pose_update_rates << pose_update.x()/last_diff_time,
                       pose_update.y()/last_diff_time,
                       pose_update.heading()/last_diff_time;
//  std::cout<<"2"<<pose_update_rates<<std::endl;
  state_mutex.unlock();
}

void DiffDrive::reset() {
  state_mutex.lock();
  last_rad_left = 0.0;
  last_rad_right = 0.0;
  last_velocity_left = 0.0;
  last_velocity_right = 0.0;
  state_mutex.unlock();
}

void DiffDrive::getWheelJointStates(float &wheel_left_angle, float &wheel_left_angle_rate,
                                    float &wheel_right_angle, float &wheel_right_angle_rate) {
  state_mutex.lock();
  wheel_left_angle = last_rad_left;
  wheel_right_angle = last_rad_right;
  wheel_left_angle_rate = last_velocity_left;
  wheel_right_angle_rate = last_velocity_right;
  state_mutex.unlock();
}

void DiffDrive::setVelocityCommands(const float &vx, const float &wz) {
  // vx: in m/s
  // wz: in rad/s
  std::vector<float> cmd_vel;
  cmd_vel.push_back(vx);
  cmd_vel.push_back(wz);
  point_velocity = cmd_vel;
}

void DiffDrive::velocityCommands(const float &vx, const float &wz) {
  // vx: in m/s
  // wz: in rad/s
  velocity_mutex.lock();
  linear_velocity = vx;
  angular_velocity = wz;
  velocity_mutex.unlock();
    return;
}


std::vector<float> DiffDrive::pointVelocity() const {
  return point_velocity;
}

std::vector<float> DiffDrive::velocityCommands() {
  velocity_mutex.lock();
  std::vector<float> cmd(2);
  cmd[0] = linear_velocity;  // In [m/s]
  cmd[1] = angular_velocity*180/ecl::pi; // In [degree/s]
//  cmd[1] = angular_velocity; // In [rad/s]
  velocity_mutex.unlock();
  return cmd;
}

short DiffDrive::bound(const float &value) {
  if (value > static_cast<float>(SHRT_MAX)) return SHRT_MAX;
  if (value < static_cast<float>(SHRT_MIN)) return SHRT_MIN;
  return static_cast<short>(value);
}

} // namespace xbot
