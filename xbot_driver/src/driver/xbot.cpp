/**
 * @file src/driver/xbot.cpp
 *
 * @brief Implementation for the xbot device driver.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <cmath>
#include <fstream>
#include <ecl/math.hpp>
#include <ecl/geometry/angle.hpp>
#include <ecl/time/sleep.hpp>
#include <ecl/converters.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/geometry/angle.hpp>
#include <ecl/time/timestamp.hpp>
#include <stdexcept>
#include "../../include/xbot_driver/xbot.hpp"
#include "../../include/xbot_driver/packet_handler/payload_headers.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace xbot
{

/*****************************************************************************
 ** Implementation [PacketFinder]
 *****************************************************************************/

bool PacketFinder::checkSum()
{
  unsigned int packet_size(buffer.size());
  unsigned char cs(0);
  for (unsigned int i = 2; i < packet_size; i++)
  {
    cs ^= buffer[i];
  }
  return cs ? false : true;
}

/*****************************************************************************
 ** Implementation [Initialisation]
 *****************************************************************************/

Xbot::Xbot() :
    shutdown_requested(false)
    , is_enabled(false)
    , is_connected(false)
    , is_alive(false)
    , heading_offset(0.0)
    , HeightPercent(0)
    , CameraDegree(0)
    , PlatformDegree(0)
    , Power(1)

{

}

/**
 * Shutdown the driver - make sure we wait for the thread to finish.
 */
Xbot::~Xbot()
{
  disable();
  shutdown_requested = true; // thread's spin() will catch this and terminate
  thread.join();
  resetXbotState();
  sig_debug.emit("Device: xbot driver terminated.");
}

void Xbot::init(Parameters &parameters) throw (ecl::StandardException)
{

  if (!parameters.validate())
  {
    throw ecl::StandardException(LOC, ecl::ConfigurationError, "Xbot's parameter settings did not validate.");
  }
  this->parameters = parameters;
  std::string sigslots_namespace = parameters.sigslots_namespace;

  // connect signals
  sig_stream_data.connect(sigslots_namespace + std::string("/stream_data"));
  sig_raw_data_command.connect(sigslots_namespace + std::string("/raw_data_command"));
  sig_raw_data_stream.connect(sigslots_namespace + std::string("/raw_data_stream"));
  //sig_serial_timeout.connect(sigslots_namespace+std::string("/serial_timeout"));

  sig_debug.connect(sigslots_namespace + std::string("/ros_debug"));
  sig_info.connect(sigslots_namespace + std::string("/ros_info"));
  sig_warn.connect(sigslots_namespace + std::string("/ros_warn"));
  sig_error.connect(sigslots_namespace + std::string("/ros_error"));

  try {
    serial.open(parameters.device_port, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);  // this will throw exceptions - NotFoundError, OpenError
    is_connected = true;
    serial.block(4000); // blocks by default, but just to be clear!
  }
  catch (const ecl::StandardException &e)
  {
    if (e.flag() == ecl::NotFoundError) {
      sig_warn.emit("device does not (yet) available, is the usb connected?."); // not a failure mode.
    } else {
      throw ecl::StandardException(LOC, e);
    }
  }

  ecl::PushAndPop<unsigned char> stx(2, 0);
  ecl::PushAndPop<unsigned char> etx(1);
  stx.push_back(0xaa);
  stx.push_back(0x55);
  packet_finder.configure(sigslots_namespace, stx, etx, 1, 256, 1, true);
  acceleration_limiter.init(parameters.enable_acceleration_limiter);



  thread.start(&Xbot::spin, *this);
}

/*****************************************************************************
 ** Implementation [Runtime]
 *****************************************************************************/
/**
 * Usually you should call the getXXX functions from within slot callbacks
 * connected to this driver's signals. This ensures that data is not
 * overwritten inbetween getXXX calls as it all happens in the serial device's
 * reading thread (aye, convoluted - apologies for the multiple robot and multiple
 * developer adhoc hacking over 4-5 years for hasty demos on pre-xbot robots.
 * This has generated such wonderful spaghetti ;).
 *
 * If instead you just want to poll xbot, then you should lock and unlock
 * the data access around any getXXX calls.
 */
void Xbot::lockDataAccess() {
  data_mutex.lock();
}

/**
 * Unlock a previously locked data access privilege.
 * @sa lockDataAccess()
 */
void Xbot::unlockDataAccess() {
  data_mutex.unlock();
}

/**
 * @brief Performs a scan looking for incoming data packets.
 *
 * Sits on the device waiting for incoming and then parses it, and signals
 * that an update has occured.
 *
 * Or, if in simulation, just loopsback the motor devices.
 */

void Xbot::spin()
{
  ecl::TimeStamp last_signal_time;
  ecl::Duration timeout(0.1);
  unsigned char buf[256];


//  int buf_size=0;


  /*********************
   ** Simulation Params
   **********************/

  while (!shutdown_requested)
  {
    /*********************
     ** Checking Connection
     **********************/
    if ( !serial.open() ) {
      try {
        // this will throw exceptions - NotFoundError is the important one, handle it
        serial.open(parameters.device_port, ecl::BaudRate_115200, ecl::DataBits_8, ecl::StopBits_1, ecl::NoParity);
        sig_info.emit("device is connected.");
        is_connected = true;
        serial.block(4000); // blocks by default, but just to be clear!
      }
      catch (const ecl::StandardException &e)
      {
        // windows throws OpenError if not connected
        if (e.flag() == ecl::NotFoundError) {
          sig_info.emit("device does not (yet) available on this port, waiting...");
        } else if (e.flag() == ecl::OpenError) {
          sig_info.emit("device failed to open, waiting... [" + std::string(e.what()) + "]");
        } else {
          // This is bad - some unknown error we're not handling! But at least throw and show what error we came across.
          throw ecl::StandardException(LOC, e);
        }
        ecl::Sleep(5)(); // five seconds
        is_connected = false;
        is_alive = false;
        continue;
      }
    }

    /*********************
     ** Read Incoming
     **********************/
    int n = serial.read((char*)buf, 1);
//    std::cout<<"n: "<<n<<std::endl;

    if (n == 0)
    {
      if (is_alive && ((ecl::TimeStamp() - last_signal_time) > timeout))
      {
        is_alive = false;
        sig_debug.emit("Timed out while waiting for incoming bytes.");
      }
      continue;
    }
    else
    {
      std::ostringstream ostream;
      ostream << "xbot_node : serial_read(" << n << ")"
        << ", packet_finder.numberOfDataToRead(" << packet_finder.numberOfDataToRead() << ")";
      //sig_debug.emit(ostream.str());
      // might be useful to send this to a topic if there is subscribers
    }
//    buf_size+=n;

    bool find_packet = packet_finder.update(buf, n);


    if (find_packet) // this clears packet finder's buffer and transfers important bytes into it
    {

      PacketFinder::BufferType local_buffer;
      packet_finder.getBuffer(local_buffer); // get a reference to packet finder's buffer.
      sig_raw_data_stream.emit(local_buffer);

      packet_finder.getPayload(data_buffer);// get a reference to packet finder's buffer.

      lockDataAccess();
      std::cout<<"data_buffer.size:"<<data_buffer.size()<<std::endl;


      while (data_buffer.size() >0)
      {

//        std::cout << "header_id: " << (unsigned int)data_buffer[0] << " | "<<std::endl;
//        std::cout << "length: " << (unsigned int)data_buffer[1] << " | ";
//        std::cout << "remains: " << data_buffer.size() << " | ";
//        std::cout << "local_buffer: " << local_buffer.size() << " | ";
//        std::cout << std::endl;
          if( !core_sensors.deserialise(data_buffer) )
              { std::cout<<"fixed"<<std::endl;
            fixPayload(data_buffer); break; }
          sig_stream_data.emit();

      }
      //std::cout << "---" << std::endl;
      unlockDataAccess();

      is_alive = true;
      last_signal_time.stamp();
//      sig_stream_data.emit();
      sendBaseControlCommand(); // send the command packet to mainboard;
    }
    else
    {
      // watchdog
      if (is_alive && ((ecl::TimeStamp() - last_signal_time) > timeout))
      {
        is_alive = false;
        // do not call here the event manager update, as it generates a spurious offline state
      }
    }
  }
  sig_error.emit("Driver worker thread shutdown!");
}

void Xbot::fixPayload(ecl::PushAndPop<unsigned char> & byteStream)
{
  if (byteStream.size() < 3 ) { /* minimum size of sub-payload is 3; header_id, length, data */
    byteStream.clear();
  } else {
    std::stringstream ostream;
    unsigned int header_id = static_cast<unsigned int>(byteStream.pop_front());
    unsigned int length = static_cast<unsigned int>(byteStream.pop_front());
    unsigned int remains = byteStream.size();
    unsigned int to_pop;

    ostream << "[" << header_id << "]";
    ostream << "[" << length << "]";

    ostream << "[";
    ostream << std::setfill('0') << std::uppercase;
    ostream << std::hex << std::setw(2) << header_id << " " << std::dec;
    ostream << std::hex << std::setw(2) << length << " " << std::dec;

    if (remains < length) to_pop = remains;
    else                  to_pop = length;

    for (unsigned int i = 0; i < to_pop; i++ ) {
      unsigned int byte = static_cast<unsigned int>(byteStream.pop_front());
      ostream << std::hex << std::setw(2) << byte << " " << std::dec;
    }
    ostream << "]";

  }
}


/*****************************************************************************
 ** Implementation [Human Friendly Accessors]
 *****************************************************************************/

float Xbot::getHeading() const
{
  ecl::Angle<float>heading;
//    float heading;
  // raw data angles are in tens of a degree, convert to radians.
  heading = (static_cast<float>(imu_sensors.data.yaw) / 10.0)*ecl::pi / 180.0;
  std::cout<<"heading:"<<heading<<" | heading_offset:"<<heading_offset<<std::endl;

  return ecl::wrap_angle(heading - heading_offset);
}

int Xbot::getDebugSensors() const
{
    return (static_cast<int>(core_sensors.data.front_left_encoder));
}
float Xbot::getAngularVelocity() const
{
  // raw data angles are in hundredths of a degree, convert to radians.
  return (static_cast<float>(core_sensors.data.yaw) / 10.0) * ecl::pi / 180.0;
}



void Xbot::resetXbotState()
{
    setLiftControl(0);
    setPowerControl(1);
    setCloudCameraControl(0,0);
}

/*****************************************************************************
 ** Implementation [Raw Data Accessors]
 *****************************************************************************/

void Xbot::resetOdometry()
{
  diff_drive.reset();

  // Issue #274: use current imu reading as zero heading to emulate reseting gyro
  heading_offset = (static_cast<float>(core_sensors.data.yaw) / 10.0) * ecl::pi / 180.0;
}

void Xbot::getWheelJointStates(float &wheel_left_angle, float &wheel_left_angle_rate, float &wheel_right_angle,
                                 float &wheel_right_angle_rate)
{
  diff_drive.getWheelJointStates(wheel_left_angle, wheel_left_angle_rate, wheel_right_angle, wheel_right_angle_rate);
}

/**
 * @brief Use the current sensor data (encoders and gyro) to calculate an update for the odometry.
 *
 * This fuses current sensor data with the last updated odometry state to produce the new
 * odometry state. This will be usually done in the slot callback to the stream_data signal.
 *
 * It is important that this is called every time a data packet is received from the robot.
 *
 * @param pose_update : return the pose updates in this variable.
 * @param pose_update_rates : return the pose update rates in this variable.
 */
void Xbot::updateOdometry(ecl::Pose2D<double> &pose_update, ecl::linear_algebra::Vector3d &pose_update_rates)
{
  diff_drive.update(core_sensors.data.timestamp, core_sensors.data.front_left_encoder, core_sensors.data.front_right_encoder,
                      pose_update, pose_update_rates);
}

/*****************************************************************************
 ** Commands
 *****************************************************************************/



void Xbot::setBaseControl(const float &linear_velocity, const float &angular_velocity)
{
  diff_drive.setVelocityCommands(linear_velocity, angular_velocity);

}

void Xbot::setPowerControl(const bool &power)
{
  sendCommand(Command::SetPowerControl(power));
  Power = power;
}

void Xbot::setLiftControl(const unsigned char &height_percent)
{
    sendCommand(Command::SetLiftHeightControl(height_percent));
    HeightPercent = height_percent;

}

void Xbot::setCloudCameraControl(const unsigned char &platform_degree, const unsigned char &camera_degree)
{
    sendCommand(Command::SetPlatformAndCameraControl(platform_degree+90,(camera_degree+45)*2));
    PlatformDegree = platform_degree;
    CameraDegree = camera_degree;

}

void Xbot::resetXbot()
{
    setBaseControl(0,0);
    setLiftControl(0);
    setPowerControl(1);
    setCloudCameraControl(0,0);
}

void Xbot::sendBaseControlCommand()
{
  std::vector<float> velocity_commands_received;
  if( acceleration_limiter.isEnabled() ) {
    velocity_commands_received=acceleration_limiter.limit(diff_drive.pointVelocity());
  } else {
    velocity_commands_received=diff_drive.pointVelocity();
  }
  diff_drive.velocityCommands(velocity_commands_received);
  std::vector<float> velocity_commands = diff_drive.velocityCommands();
  std::cout << "linear_velocity: " << velocity_commands[0] << ", angular_velocity: " << velocity_commands[1] << std::endl;
  if ((velocity_commands[0]!=0) || (velocity_commands[1]!=0)){
      sendCommand(Command::SetVelocityControl(velocity_commands[0], velocity_commands[1]));
  }

//  //experimental; send raw control command and received command velocity
//  velocity_commands_debug=velocity_commands;
//  velocity_commands_debug.push_back((short)(velocity_commands_received[0]*1000.0));
//  velocity_commands_debug.push_back((short)(velocity_commands_received[1]*1000.0));
//  sig_raw_control_command.emit(velocity_commands_debug);
}

/**
 * @brief Send the prepared command to the serial port.
 *
 * Need to be a bit careful here, because we have no control over how the user
 * is calling this - they may be calling from different threads (this is so for
 * xbot_node), so we mutex protect it here rather than relying on the user
 * to do so above.
 *
 * @param command : prepared command template (see Command's static member functions).
 */
void Xbot::sendCommand(Command command)
{
  if( !is_alive || !is_connected ) {
    //need to do something
    sig_debug.emit("Device state is not ready yet.");
    if( !is_alive     ) sig_debug.emit(" - Device is not alive.");
    if( !is_connected ) sig_debug.emit(" - Device is not connected.");
    //std::cout << is_enabled << ", " << is_alive << ", " << is_connected << std::endl;
    return;
  }
  command_mutex.lock();
  xbot_command.resetBuffer(command_buffer);

  if (!command.serialise(command_buffer))
  {
    sig_error.emit("command serialise failed.");
  }
  command_buffer[2] = command_buffer.size() - 3;
  unsigned char checksum = 0;
  for (unsigned int i = 2; i < command_buffer.size(); i++)
    checksum ^= (command_buffer[i]);

  command_buffer.push_back(checksum);
  //check_device();
//  for (int i = 0;i < command_buffer.size();i++)
//    {
//  //      std::cout <<std::hex<<command_buffer[i]<<std::endl;
//        printf("%02x ",command_buffer[i]);

//    }
    std::cout<<std::endl;
  serial.write((const char*)&command_buffer[0], command_buffer.size());

  sig_raw_data_command.emit(command_buffer);
  command_mutex.unlock();
}

bool Xbot::enable()
{
  is_enabled = true;
  return true;
}

bool Xbot::disable()
{
  setBaseControl(0.0f, 0.0f);
  sendBaseControlCommand();
  is_enabled = false;
  return true;
}

/**
 * @brief Print a list of all relevant sigslot connections.
 *
 * This includes both the xbot driver signals as well as externally
 * connected slots. Useful for when you need to check if any of your
 * connections are dangling (often happens when you typo
 * the name of the sigslots connection).
 */

} // namespace xbot
