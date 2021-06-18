#include <ros/ros.h>
#include <iostream>
#include "ros/xbot_talker_ros.h"
int main(int argc, char** argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "nlp_sample");
  NLPModuleRos nlp_ros;
  nlp_ros.init();
  ros::spin();
}
