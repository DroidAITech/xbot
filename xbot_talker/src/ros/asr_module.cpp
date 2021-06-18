#include <ros/ros.h>
#include "asr/xunfei/xfei_speech_recog.h"
#include <iostream>
#include "common_config.h"
#include "ros/xbot_talker_ros.h"
int main(int argc, char** argv)
{
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "asr_sample");

  ASRModuleRos asr_ros;
  asr_ros.init();
  ros::spin();

}
