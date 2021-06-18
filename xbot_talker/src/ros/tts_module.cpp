#include "tts/text_to_speech.h"
#include <ros/ros.h>
#include <iostream>
#include "ros/xbot_talker_ros.h"
int main(int argc, char** argv)
{
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "tts_sample");
  TTSModuleRos tts_ros;
  tts_ros.init();
  ros::spin();

}
