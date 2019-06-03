/**                                                                           !
  * @file /xbot_driver/src/test/simple_loop.cpp
  *
  * @brief Example/test program with simple loop.
  *
  * It provides simple example of how interact with xbot by using c++ without ROS.
 **/

/*****************************************************************************
 * Includes
 ****************************************************************************/

#include <csignal>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/geometry/pose2d.hpp>
#include <ecl/linear_algebra.hpp>
#include "xbot_driver/xbot.hpp"

/*****************************************************************************
** Classes
*****************************************************************************/

class XbotManager {
public:
  XbotManager() :
    dx(0.0), dth(0.0),
    slot_stream_data(&XbotManager::processStreamData, *this)
  {
    xbot::Parameters parameters;
    parameters.sigslots_namespace = "/xbot";
    parameters.device_port = "/dev/xbot";
    parameters.enable_acceleration_limiter = false;
    xbot.init(parameters);
    xbot.enable();
    slot_stream_data.connect("/xbot/stream_data");
  }

  ~XbotManager() {
    xbot.setBaseControl(0,0); // linear_velocity, angular_velocity in (m/s), (rad/s)
    xbot.disable();
  }

  void processStreamData() {
    ecl::Pose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    xbot.updateOdometry(pose_update, pose_update_rates);
    pose *= pose_update;
    dx += pose_update.x();
    dth += pose_update.heading();
    //std::cout << dx << ", " << dth << std::endl;
    std::cout << xbot.getHeading() << ", " << pose.heading() << std::endl;
    //std::cout << "[" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
    processMotion();
  }

  // Generate square motion
  void processMotion() {
//    if (dx >= 1.0 && dth >= ecl::pi/2.0) { dx=0.0; dth=0.0; xbot.setBaseControl(0.0, 0.0); return; }
//    else if (dx >= 1.0) { xbot.setBaseControl(0.0, 10.3); return; }
//    else { xbot.setBaseControl(0.4, 0.0); return; }
      xbot.setBaseControl(0.0, -1.1);
//      xbot.setLiftControl(0);
//      xbot.setPlatformCameraControl(10,20);
//      xbot.resetXbot();
      return;
  }

  ecl::Pose2D<double> getPose() {
    return pose;
  }

private:
  double dx, dth;
  ecl::Pose2D<double> pose;
  xbot::Xbot xbot;
  ecl::Slot<> slot_stream_data;
};

/*****************************************************************************
** Signal Handler
*****************************************************************************/

bool shutdown_req = false;
void signalHandler(int signum) {
  shutdown_req = true;
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  signal(SIGINT, signalHandler);

  std::cout << "Demo : Example of simple control loop." << std::endl;
  XbotManager xbot_manager;



  ecl::Sleep sleep(1);
  ecl::Pose2D<double> pose;
  try {
    while (!shutdown_req){
      sleep();
      pose = xbot_manager.getPose();
//      std::cout << "current pose: [" << pose.x() << ", " << pose.y() << ", " << pose.heading() << "]" << std::endl;
    }
  } catch ( ecl::StandardException &e ) {
    std::cout << e.what();
  }
  return 0;
}
