#include "asr/xunfei/xfei_speech_recog.h"
#include <ros/ros.h>
#include <iostream>
#include "common_config.h"
#include "ros/xbot_talker_ros.h"
int main(int argc, char** argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "awaken_sample");
  AwakenModuleRos awaken_ros;
  awaken_ros.init();
  ros::spin();
}
