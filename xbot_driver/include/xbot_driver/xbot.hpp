/**
 * @file include/xbot_driver/xbot.hpp
 *
 * @brief Device driver core interface.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef XBOT_HPP_
#define XBOT_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <iomanip>
#include <ecl/config.hpp>
#include <ecl/threads.hpp>
#include <ecl/devices.hpp>
#include <ecl/threads/mutex.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "parameters.hpp"
#include "command.hpp"
#include "modules.hpp"
#include "packets.hpp"
#include "packet_handler/packet_finder.hpp"
#include "macros.hpp"


/*****************************************************************************
** Extern Templates
*****************************************************************************/

#ifdef ECL_IS_WIN32
  /* Help windows create common instances of sigslots across xbot dll
   * and end user program (otherwise it creates two separate variables!) */
  EXP_TEMPLATE template class xbot_PUBLIC ecl::SigSlotsManager<>;
  EXP_TEMPLATE template class xbot_PUBLIC ecl::SigSlotsManager<const xbot::VersionInfo&>;
  EXP_TEMPLATE template class xbot_PUBLIC ecl::SigSlotsManager<const std::string&>;
  EXP_TEMPLATE template class xbot_PUBLIC ecl::SigSlotsManager<xbot::Command::Buffer&>;
  EXP_TEMPLATE template class xbot_PUBLIC ecl::SigSlotsManager<xbot::PacketFinderBase::BufferType&>;
#endif

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace xbot
{

/*****************************************************************************
 ** Definitions
 *****************************************************************************/

union union_sint16
{
  short word;
  unsigned char byte[2];
};

/*****************************************************************************
** Parent Interface
*****************************************************************************/

class PacketFinder : public PacketFinderBase
{
public:
  virtual ~PacketFinder() {}
  bool checkSum();
};

/*****************************************************************************
 ** Interface [Xbot]
 *****************************************************************************/
/**
 * @brief  The core xbot driver class.
 *
 * This connects to the outside world via sigslots and get accessors.
 **/
class xbot_PUBLIC Xbot
{
public:
  Xbot();
  ~Xbot();

  /*********************
   ** Configuration
   **********************/
  void init(Parameters &parameters) throw (ecl::StandardException);
  bool isAlive() const { return is_alive; } /**< Whether the connection to the robot is alive and currently streaming. **/
  bool isShutdown() const { return shutdown_requested; } /**< Whether the worker thread is alive or not. **/
  bool isEnabled() const { return is_enabled; } /**< Whether the motor power is enabled or disabled. **/
  bool enable(); /**< Enable power to the motors. **/
  bool disable(); /**< Disable power to the motors. **/
  void shutdown() { shutdown_requested = true; } /**< Gently terminate the worker thread. **/
  void resetXbotState();

  /******************************************
  ** Packet Processing
  *******************************************/
  void spin();
  void fixPayload(ecl::PushAndPop<unsigned char> & byteStream);

  /******************************************
  ** Getters - Data Protection
  *******************************************/
  void lockDataAccess();
  void unlockDataAccess();

  /******************************************
  ** Getters - User Friendly Api
  *******************************************/
  /* Be sure to lock/unlock the data access (lockDataAccess and unlockDataAccess)
   * around any getXXX calls - see the doxygen notes for lockDataAccess. */
  float getHeading() const;
  int getDebugSensors() const;
  float getAngularVelocity() const;
  unsigned char getHeightPercent() {return HeightPercent;}
  unsigned char getCameraDegree(){return CameraDegree;}
  unsigned char getPlatformDegree(){return PlatformDegree;}
  bool getPowerState(){return Power;}

  /******************************************
  ** Getters - Raw Data Api
  *******************************************/
  /* Be sure to lock/unlock the data access (lockDataAccess and unlockDataAccess)
   * around any getXXX calls - see the doxygen notes for lockDataAccess. */
  CoreSensors::Data getCoreSensorData() const { return core_sensors.data; }
  ImuSensors::Data getImuSensorData() const {return imu_sensors.data;}
  /*********************
  ** Feedback
  **********************/
  void getWheelJointStates(float &wheel_left_angle, float &wheel_left_angle_rate,
                           float &wheel_right_angle, float &wheel_right_angle_rate);
  void updateOdometry(ecl::Pose2D<double> &pose_update,
                      ecl::linear_algebra::Vector3d &pose_update_rates);

  /*********************
  ** Soft Commands
  **********************/
  void resetOdometry();

  /*********************
  ** Hard Commands
  **********************/
  void setBaseControl(const float &linear_velocity, const float &angular_velocity);
  void setLiftControl(const unsigned char &height_percent);
  void setCloudCameraControl(const unsigned char &platform_degree, const unsigned char &camera_degree);
  void setPowerControl(const bool &power);
  void resetXbot();

private:
  /*********************
  ** Thread
  **********************/
  ecl::Thread thread;
  bool shutdown_requested; // helper to shutdown the worker thread.

  /*********************
  ** Record RobotState
  **********************/
  unsigned char HeightPercent;
  unsigned char PlatformDegree;
  unsigned char CameraDegree;
  bool Power;


  /*********************
  ** Odometry
  **********************/
  DiffDrive diff_drive;
  bool is_enabled;

  /*********************
  ** Inertia
  **********************/
  float heading_offset;

  /*********************
  ** Driver Paramters
  **********************/
  Parameters parameters;
  bool is_connected;

  /*********************
  ** Acceleration Limiter
  **********************/
  AccelerationLimiter acceleration_limiter;

  /*********************
  ** Packet Handling
  **********************/
  CoreSensors core_sensors;

  ImuSensors imu_sensors;
  ecl::Serial serial;
  PacketFinder packet_finder;
  PacketFinder::BufferType data_buffer;
  PacketFinder::BufferType buf_tmp;
  bool is_alive; // used as a flag set by the data stream watchdog


  /*********************
  ** Commands
  **********************/
  void sendBaseControlCommand();
  void sendCommand(Command command);
  ecl::Mutex command_mutex; // protection against the user calling the command functions from multiple threads
  // data_mutex is protection against reading and writing data structures simultaneously as well as
  // ensuring multiple get*** calls are synchronised to the same data update
  // refer to https://github.com/yujinrobot/xbot/issues/240
  ecl::Mutex data_mutex;
  Command xbot_command; // used to maintain some state about the command history
  Command::Buffer command_buffer;


  /*********************
  ** Signals
  **********************/
  ecl::Signal<> sig_stream_data;
  ecl::Signal<const std::string&> sig_debug, sig_info, sig_warn, sig_error;
  ecl::Signal<Command::Buffer&> sig_raw_data_command; // should be const, but pushnpop is not fully realised yet for const args in the formatters.
  ecl::Signal<PacketFinder::BufferType&> sig_raw_data_stream; // should be const, but pushnpop is not fully realised yet for const args in the formatters.
};

} // namespace xbot

#endif /* XBOT_HPP_ */
