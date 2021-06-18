#ifndef XBOT_TALKER_ROS_H_
#define XBOT_TALKER_ROS_H_
#include <std_msgs/Bool.h>
#include <iostream>
#include "awaken/awaken_offline.h"
#include "asr/baidu/asr_online.h"
#include "file_operation.h"
#include "nlp/nlp_feedback.h"
#include "tts/text_to_speech.h"
#include "xbot_talker/awaken_status.h"
#include "xbot_talker/keyword_config.h"
#include "xbot_talker/nlpdialog_config.h"
#include "xbot_talker/online_asr_result.h"
#include "xbot_talker/recog_result.h"
#include "xbot_talker/XbotTts.h"
#include "asr/xunfei/xfei_speech_recog.h"
#include "ros/ros.h"
#include "xbot_talker/EnableASR.h"
#include "xbot_talker/play.h"
#include "xbot_talker/chat.h"
#include "asr/xunfei/BuildGrammar.h"
#include "xbot_talker/CallPlay.h"
#include "xbot_talker/CallChat.h"
#include "std_msgs/Int8.h"
#include "xbot_talker/CallVersion.h"
// 与asr模块相关的ROS操作的封装类
class ASRModuleRos
{
public:
  ASRModuleRos() : offline_result_vector(2),enable_beep_(false)
  {
    getParams();
    advertiseTopics();
    subscribeTopics();
    advertiseService();
  }
  ~ASRModuleRos(){};
  std::string offline_recog_result;
  std::string baidu_online_result;
  std::string xfei_online_result;

  bool init();
  void offlineRecog();
  void onlineRecogXfei();
  void onlineRecogBaidu();

private:
  ros::NodeHandle asr_nodehandle;
  void getParams();
  void advertiseTopics();
  void subscribeTopics();
  void advertiseService();
  bool enable_beep_;
  ros::ServiceServer chat_server_,talker_chat_server_,get_version_server_;
  bool requestChat(xbot_talker::chat::Request &req, xbot_talker::chat::Response &res);
  bool requestVersion(xbot_talker::CallVersion::Request &req, xbot_talker::CallVersion::Response &res);
  bool requestTalkerChat(xbot_talker::CallChat::Request &req, xbot_talker::CallChat::Response &res);



  ros::ServiceServer keyword_config_service;
  bool requestKeywordConfig(xbot_talker::keyword_config::Request& req, xbot_talker::keyword_config::Response& res);

  /*********************
   ** Ros Publishers
   **********************/
  ros::Publisher recog_result_pub;
  ros::Publisher online_result_pub;
  ros::Publisher enable_chat_pub_;
  ros::Publisher baidu_online_recog_result_, xfei_online_recog_result_, xfei_offline_recog_result_;

  /*********************
  ** Ros Publisher Function
  **********************/
  void pubOfflineRecogResult(const std::string result, const int accuracy);
  void pubOnlineRecogResult(const std::string result);
  /*********************
   ** Ros Subscribers
   **********************/
  ros::Subscriber enable_asr_sub_, enable_beep_sub_;

  /*********************
  ** Ros Subscribers Callbacks
  **********************/
  void subscribeEnableASR(const std_msgs::Bool);
  void subscribeEnableBeep(const std_msgs::Bool);


  void subscribeAwakenStatus(const xbot_talker::awaken_status);

  /*********************
  ** ASR variable
  **********************/
  bool enable_xfei_online;
  bool enable_baidu_online;
  bool enable_xfei_offline;
  bool use_pcm_file;
  bool use_mic;
  bool enable_record_save;
  std::string asr_params_;
  std::string xunfei_online_asr_params_;
  std::string base_path;
  std::string grammar_path;
  std::string config_path;
  std::string pcm_file;
  std::string log_path;
  std::string baidu_api_key_,baidu_secret_key_;
  int audio_channel;
  int log_count_ = 0;
  int online_log_count_ = 0;
  float record_time;
  char* recog_result_json;
  std::vector<std::string> offline_result_vector;
  struct DataBuff pcm_buff = { NULL, 0 };
  FileOperation config_file;
  BuildGrammar build_grammar_;
  UserData asr_data_;
  XfeiSpeechRecog xfei_sr_offline;
  XfeiSpeechRecog xfei_sr_online;
  BaiduAsrOnline baidu_sr_online;
  CSVOperation csv_file;
  TextToSpeech tts_play_;

  // 构建语法
  bool buildASRGrammarXfei();
};

// 与nlp模块相关的ROS操作的封装类
class NLPModuleRos
{
public:
  NLPModuleRos();
  ~NLPModuleRos();
  bool init();
  void pubStartRecog();

private:
  ros::NodeHandle nlp_nodehandle;

  void advertiseTopics();
  void subscribeTopics();
  void advertiseService();

  // 根据得到的响应策略，控制机器人进行具体的响应。响应结束后返回true
  bool robotResponse();

  /*********************
   ** Ros Publishers
   **********************/
  ros::Publisher start_awaken_pub;  // 开始语音唤醒
  ros::Publisher start_asr_pub_;   // 开始语音识别
  ros::Publisher mov_control_pub_, pitch_platform_pub_, yaw_platform_pub_;   // 机器人移动控制
  ros::Publisher left_put_pub;
  ros::Publisher right_put_pub;

  ros::Publisher left_get_pub;
  ros::Publisher right_get_pub;
  ros::Publisher reset_arm_pub_,arm_lift_up_pub_,arm_put_down_pub_,gripper_control_pub_;

  ros::Publisher left_grip_pub;
  ros::Publisher right_grip_pub;

  ros::Publisher welcome_kp_pub, enable_beep_pub_;
  ros::Publisher navi_start_pub_,navi_to_pose_pub_, navi_canel_pub_,navi_continue_pub_,welcome_yes_pub;

  ros::ServiceServer dialog_config_service;

  ros::ServiceClient call_tts_;
  bool requestDialogConfig(xbot_talker::nlpdialog_config::Request& req, xbot_talker::nlpdialog_config::Response& res);
  /*********************
  ** Ros Publisher Function
  **********************/
  void pubMoveControl(const int robot_action);
  void pubArmControl(const int robot_action);
  void pubNaviControl(const int robot_action);

  void pubStartAwaken(bool enable_awake);
  /*********************
   ** Ros Subscribers
   **********************/
  ros::Subscriber recog_result_sub;
  ros::Subscriber awaken_result_sub;
  ros::Subscriber online_result_sub;
  ros::Subscriber enable_chat_sub_;


  /*********************
  ** Ros Subscribers Callbacks
  **********************/

  bool callTTServer(std::string txt);
  bool robotAction();
  void subscribeOfflineRecogResult(const xbot_talker::recog_result);
  void subscribeOnlineRecogResult(const xbot_talker::online_asr_result);
  void subscribeEnableChat(const std_msgs::Bool msg);

  /*********************
  **  NLP variable
  **********************/
  std::vector<std::string> answer_dictionary_;
  std::string base_path;
  std::string nlp_config_path;
  std::string config_path;
  std::string log_path, tuling_key_;
  int audio_channel;
  bool enable_chat_;
  int fail_count_ = 0;
  int log_count_ = 0;
  std::string tuling_answer_text_;
  ResultFeedback result_handle;
  TuLingRobot tuling_robot;
  TextToSpeech tts_play_;

};

// 与awaken模块相关的ROS操作的封装类
class AwakenModuleRos
{
public:
  AwakenModuleRos();
  ~AwakenModuleRos();
  bool init();
  void isWaken();

private:
  AwakenOffline awaken_module;
  ros::NodeHandle awaken_nodehandle;
  void getParams();
  void advertiseTopics();
  void subscribeTopics();
  void subscribeStartAwaken(const std_msgs::BoolConstPtr);
  void pubStartRecog(bool is_awaken);
  bool enable_record_save;
  std::string awaken_params_;
  std::string base_path;
  std::string config_path;
  std::string pcm_file;
  std::string awaken_mode;
  std::string audio_save_path;
  std::string log_path;
  int audio_channel;
  int record_time;
  std::string appid_;
  ros::Publisher is_awaken_pub_, enable_asr_pub_;
  ros::Subscriber start_awaken_sub_;
};

// 与tts模块相关的ROS操作的封装类
class TTSModuleRos
{
public:
  TTSModuleRos();
  ~TTSModuleRos();
  bool init();

private:
  ros::NodeHandle tts_nodehandle;

  void advertiseServices();
  /*********************
   ** Ros TTS RequestServiceClient
   **********************/
  ros::ServiceClient tts_client;

  /*********************
   ** Ros TTS ResponseServiceServer
   **********************/
  ros::ServiceServer tts_server, play_server_, talker_play_server_;

  /*********************
  ** Ros TTS ServiceServer Callbacks
  **********************/
  bool ttsCallback(xbot_talker::XbotTts::Request& req, xbot_talker::XbotTts::Response& res);
  bool playCallback(xbot_talker::play::Request &req, xbot_talker::play::Response &res);
  bool talkerPlayCallback(xbot_talker::CallPlay::Request &req, xbot_talker::CallPlay::Response &res);

  /*********************
  ** TTS variable
  **********************/
  std::string base_path;
  std::string log_path;
};
#endif
