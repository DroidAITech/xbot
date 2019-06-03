/**
 * @file src/driver/command.cpp
 *
 * @brief Implementation of the command packets.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_driver/LICENSE
**/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/xbot_driver/command.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace xbot {

/*****************************************************************************
** Static variables initialization
*****************************************************************************/

const unsigned char Command::header0 = 0xaa;
const unsigned char Command::header1 = 0x55;

/*****************************************************************************
** Implementation [Command Generators]
*****************************************************************************/






Command Command::SetVelocityControl(DiffDrive& diff_drive)
{
  Command outgoing;
  std::vector<float> velocity_commands = diff_drive.velocityCommands();
  outgoing.data.speed = velocity_commands[0];
  outgoing.data.radius = velocity_commands[1];
  outgoing.data.command = Command::BaseControl;
  return outgoing;
}

Command Command::SetVelocityControl(const float &speed, const float &radius)
{
  Command outgoing;
  outgoing.data.speed = speed;
  outgoing.data.radius = radius;
  outgoing.data.command = Command::BaseControl;
  return outgoing;
}

Command Command::SetLiftHeightControl(const unsigned char &lift_height)
{
    Command outgoing;
    outgoing.data.lift_height = lift_height;
    outgoing.data.command = Command::Lift;
    return outgoing;


}

Command Command::SetPlatformAndCameraControl(const unsigned char &platform_angle, const unsigned char &camera_angle)
{
    Command outgoing;
    outgoing.data.platform_angle = platform_angle;
    outgoing.data.camera_angle = camera_angle;
    outgoing.data.command = Command::TurnPlatformAndCamera;
    return outgoing;


}

Command Command::SetPowerControl(const bool &power_state)
{
    Command outgoing;
    outgoing.data.power_state = power_state;
    outgoing.data.command = Command::Power;
    return outgoing;
}

/*****************************************************************************
** Implementation [Serialisation]
*****************************************************************************/
/**
 * Clears the command buffer and resets the header.
 */
void Command::resetBuffer(Buffer& buffer) {
  buffer.clear();
  buffer.resize(64);
  buffer.push_back(Command::header0);
  buffer.push_back(Command::header1);
  buffer.push_back(0); // just initialise, we usually write in the payload here later (size of payload only, not stx, not etx, not length)
}

bool Command::serialise(ecl::PushAndPop<unsigned char> & byteStream)
{
  // need to be sure we don't pass through an emum to the Trans'd buildBytes functions.
  unsigned char cmd = static_cast<unsigned char>(data.command);
  unsigned char lift_label = 1;
  switch (data.command)
  {
    case BaseControl:
      buildBytes(cmd, byteStream);
      buildBytes(data.speed, byteStream);
      buildBytes(data.speed, byteStream);
      buildBytes(data.radius, byteStream);
      break;
    case Power:
      buildBytes(cmd, byteStream);
      buildBytes(data.power_state, byteStream);
      break;
    case Lift:
      buildBytes(cmd, byteStream);
      buildBytes(lift_label, byteStream);
      buildBytes(data.lift_height, byteStream);
      break;
    case TurnPlatformAndCamera:
      buildBytes(cmd, byteStream);
      buildBytes(data.platform_angle, byteStream);
      buildBytes(data.camera_angle, byteStream);
      break;
    default:
      return false;
      break;
  }
  return true;
}



} // namespace xbot
