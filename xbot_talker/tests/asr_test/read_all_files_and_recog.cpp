#include <ros/ros.h>
#include <iostream>
#include <thread>
#include "asr/baidu/asr_online.h"
#include "common_config.h"
#include "file_operation.h"
#include "asr/xunfei/xfei_speech_recog.h"

XfeiSpeechRecog asr_module;
BaiduAsrOnline asr_online_module;
int count = 0;
std::string base_path;
std::string log_path;

// 离线语音识别线程
void offlineRecog()
{
  char* recog_result;
  asr_module.initAsr();
  recog_result = asr_module.dataLoopRecog();
  ROS_INFO_STREAM("Offline asr result is :" << recog_result << std::endl);
  std::ofstream result_log(log_path + "/test_result_log.txt", std::ios::app);
  if (result_log.is_open())
  {
    result_log << count << "\n" << recog_result << std::endl;
  }
  else
  {
    ROS_ERROR_STREAM("LOG ERROR");
  }
  result_log.close();

  asr_module.uninitAsr();
}

// 在线语音识别线程
void onlineRecog()
{
  char* recog_result_online;
  recog_result_online = asr_online_module.runAsrAndRecog();
  ROS_INFO_STREAM("Online asr result is :" << recog_result_online << std::endl);
  std::ofstream result_log(log_path + "/test_online_result_log.txt", std::ios::app);
  if (result_log.is_open())
  {
    result_log << count << "\n" << recog_result_online << std::endl;
  }
  else
  {
    ROS_ERROR_STREAM("LOG ERROR");
  }
  result_log.close();
  asr_online_module.uninitAsr();
}
int main(int argc, char** argv)
{
  setlocale(LC_ALL, "");

  bool enable_online;
  bool enable_offline;
  std::string asr_params_;
  std::string grammar_path;
  std::string pcm_file;
  int audio_channel;
  std::string test_dir_path,baidu_api_key,baidu_secret_key;
  ros::init(argc, argv, "asr_sample");

  ros::NodeHandle asr_nodehandle;
  // 获取参数
  asr_nodehandle.param("/asr_sample/enable_online", enable_online, bool(false));
  asr_nodehandle.param("/asr_sample/enable_offline", enable_offline, bool(true));

  asr_nodehandle.param("/asr_sample/base_path", base_path, std::string("~/catkin_ws/src/xbot_talker"));
  asr_nodehandle.param("/asr_sample/test_dir_path", test_dir_path, std::string("~/catkin_ws/src/xbot_talker"));
  asr_nodehandle.param("/asr_sample/grammar_path", grammar_path,
                       std::string("~/catkin_ws/src/xbot_talker/cache/grammar_config"));
  asr_nodehandle.param("/asr_sample/pcm_file", pcm_file,
                       std::string("~/catkin_ws/src/xbot_talker/defaultconfig/audio/nihao_test.pcm"));

  asr_nodehandle.param("/asr_sample/log_path", log_path, std::string("~/catkin_ws/src/xbot_talker/cache/log"));

  asr_nodehandle.param("/asr_sample/audio_channel", audio_channel, int(1));
  asr_nodehandle.param("/asr_sample/baidu_api_key", baidu_api_key, std::string("kVcnfD9iW2XVZSMaLMrtLYIz"));
  asr_nodehandle.param("/asr_sample/baidu_secret_key", baidu_secret_key, std::string("O9o1O213UgG5LFn0bDGNtoRN3VWl2du6"));


  if (enable_offline == true)
  {
    // 科大讯飞asr登录以及语法参数配置
    CommonConfig& xunfei_config = CommonConfig::get_instance();
    xunfei_config.loginToXunfei(base_path);
    asr_params_ = xunfei_config.configGramParas(base_path, grammar_path);
  }
  // 设置百度在线语音识别相关参数
  if (enable_online == true)
  {
    asr_online_module.initAndConfigAsr();
    asr_online_module.speechGetToken();
  }
  std::cout << "Test dir is :" << test_dir_path << std::endl;
  FileOperation test_file;
  std::vector<std::string> files;
  test_file.getAllFilesName(test_dir_path, files);

  for (int i = 0; i < files.size(); i++)
  {
    asr_module.setAsrParams(base_path, test_dir_path + "/" + files[i], asr_params_, audio_channel);
    asr_online_module.setAsrParams(baidu_api_key, baidu_secret_key, base_path, test_dir_path + "/" + files[i], audio_channel);

    if (enable_offline && enable_online)
    {
      asr_module.getPcmFileData();
      asr_online_module.getPcmFileData();

      std::thread offlineThread(offlineRecog);
      std::thread onlineThread(onlineRecog);
      onlineThread.join();
      offlineThread.join();
    }
    else if (enable_offline == true)
    {
      asr_module.getPcmFileData();

      std::thread offlineThread(offlineRecog);
      offlineThread.join();
    }
    else if (enable_online == true)
    {
      asr_online_module.getPcmFileData();

      std::thread onlineThread(onlineRecog);
      onlineThread.join();
    }
    count++;
    std::cout << files[i] << std::endl;
  }
  return 0;
}
