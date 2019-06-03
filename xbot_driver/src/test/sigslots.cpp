/**
 * @file /xbot_driver/src/test/sigslots.cpp
 *
 * @brief Example/test program for xbot sigslots.
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <iostream>
#include <xbot_driver/xbot.hpp>

/*****************************************************************************
** Classes
*****************************************************************************/

class XbotManager {
public:
  XbotManager() :
      slot_stream_data(&XbotManager::processStreamData, *this) // establish the callback
  {
    xbot::Parameters parameters;
    parameters.sigslots_namespace = "/mobile_base"; // configure the first part of the sigslot namespace
    parameters.device_port = "/dev/xbot";         // the serial port to connect to (windows COM1..)
    // configure other parameters here
    xbot.init(parameters);
    slot_stream_data.connect("/mobile_base/stream_data");
  }

  void spin() {
    ecl::Sleep sleep(1);
    while ( true ) {
      sleep();
    }
  }

  /*
   * Called whenever the xbot receives a data packet. Up to you from here to process it.
   *
   * Note that special processing is done for the various events which discretely change
   * state (bumpers, cliffs etc) and updates for these are informed via the xxxEvent
   * signals provided by the xbot driver.
   */
  void processStreamData() {
    xbot::CoreSensors::Data data = xbot.getCoreSensorData();
    std::cout << "Encoders [" <<  data.front_left_encoder << "," << data.front_right_encoder << "]" << std::endl;
  }

private:
  xbot::Xbot xbot;
  ecl::Slot<> slot_stream_data;
};

/*****************************************************************************
** Main
*****************************************************************************/

int main() {
  XbotManager xbot_manager;
  xbot_manager.spin();
  return 0;
}
