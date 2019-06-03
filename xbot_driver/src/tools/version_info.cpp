/**                                                                           !
  * @file /xbot_driver/src/tools/version_info.cpp
  *
  * @brief Tools/utility program to retriving version info. of xbot.
  *
 **/

/*****************************************************************************
 * Includes
 ****************************************************************************/

#include <string>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/command_line.hpp>
#include "xbot_driver/xbot.hpp"

/*****************************************************************************
** Classes
*****************************************************************************/

class XbotManager {
public:
  XbotManager(const std::string &device_port) :
    acquired(false)
  {
    xbot::Parameters parameters;
    parameters.sigslots_namespace = "/xbot"; // configure the first part of the sigslot namespace
    parameters.device_port = device_port;    // the serial port to connect to (windows COM1..)
    xbot.init(parameters);
    xbot.enable();
  }

  ~XbotManager() {
    xbot.disable();
  }


  bool isAcquired() { return acquired; }
  std::string& getHardwareVersion() { return hardware; }
  std::string& getFirmwareVersion() { return firmware; }
  std::string& getSoftwareVersion() { return software; }
  std::string& getUDID() { return udid; }

private:
  volatile bool acquired;
  xbot::Xbot xbot;
  std::string hardware, firmware, software, udid;
};

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  ecl::CmdLine cmd_line("version_info program", ' ', "0.2");
  ecl::UnlabeledValueArg<std::string> device_port("device_port", "Path to device file of serial port to open, connected to the xbot", false, "/dev/xbot", "string");
  cmd_line.add(device_port);
  cmd_line.parse(argc, argv);
  //std::cout << "device_port: " << device_port.getValue() << std::endl;

  std::cout << "Version Info:" << std::endl;
  XbotManager xbot_manager(device_port.getValue());

  while (!xbot_manager.isAcquired());
  std::cout << " * Hardware Version: " << xbot_manager.getHardwareVersion() << std::endl;
  std::cout << " * Firmware Version: " << xbot_manager.getFirmwareVersion() << std::endl;
  std::cout << " * Software Version: " << xbot_manager.getSoftwareVersion() << std::endl;
  std::cout << " * Unique Device ID: " << xbot_manager.getUDID() << std::endl;

  return 0;
}
