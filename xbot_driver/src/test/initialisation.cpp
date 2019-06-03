/**
 * @file /xbot_driver/src/test/initialisation.cpp
 *
 * @brief Demo program for xbot initialisation.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <xbot_driver/xbot.hpp>
#include <ecl/time.hpp>

class XbotManager {
public:
  XbotManager() {
    xbot::Parameters parameters;
    // change the default device port from /dev/xbot to /dev/ttyUSB0
    parameters.device_port = "/dev/ttyUSB0";
    // Other parameters are typically happy enough as defaults
    // namespaces all sigslot connection names under this value, only important if you want to
    parameters.sigslots_namespace = "/xbot";
    // Most people will prefer to do their own velocity smoothing/acceleration limiting.
    // If you wish to utilise xbot's minimal acceleration limiter, set to true
    parameters.enable_acceleration_limiter = false;
    // If your battery levels are showing significant variance from factory defaults, adjust thresholds.
    // This will affect the led on the front of the robot as well as when signals are emitted by the driver.
    // initialise - it will throw an exception if parameter validation or initialisation fails.
    try {
      xbot.init(parameters);
    } catch ( ecl::StandardException &e ) {
      std::cout << e.what();
    }
  }
private:
  xbot::Xbot xbot;
};

int main() {
  XbotManager xbot_manager;
  ecl::Sleep()(500);
  return 0;
}
