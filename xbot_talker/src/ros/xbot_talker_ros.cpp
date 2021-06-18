#include "ros/xbot_talker_ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <thread>
#include "common_config.h"
#include "asr/xunfei/xfei_speech_recog.h"
/**************************************************
*************   Ros ASR Module  *******************
***************************************************/

bool ASRModuleRos::init()
{

  // 科大讯飞asr登录以及语法参数配置
  CommonConfig& xunfei_config = CommonConfig::get_instance();
  xunfei_config.loginToXunfei(base_path);
  if (enable_xfei_offline == true)
  {
    buildASRGrammarXfei();
    sleep(1);
    asr_params_ = xunfei_config.configGramParas(base_path, grammar_path);
    xfei_sr_offline.setAsrParams(base_path, pcm_file, asr_params_, audio_channel);
  }
  // 设置科大讯飞在线语音识别相关参数
  if (enable_xfei_online == true)
  {
    xunfei_online_asr_params_ = "sub = iat, domain = iat, language = zh_cn, "
                                "accent = mandarin, sample_rate = 16000, "
                                "result_type = plain, result_encoding = utf8";
    xfei_sr_online.setAsrParams(base_path, pcm_file, xunfei_online_asr_params_, audio_channel);
  }
  if (enable_baidu_online == true)
  {
    // 设置百度在线语音识别相关参数
    baidu_sr_online.setAsrParams(baidu_api_key_, baidu_secret_key_, base_path, pcm_file, audio_channel);
    // 百度语音识别模块的初始化,设置asr相关参数
    baidu_sr_online.initAndConfigAsr();
    // 获取百度在线语音识别的Token
    baidu_sr_online.speechGetToken();
  }

  ROS_INFO_STREAM("--------Speech recognition module is ready to be waken up!--------");
}


bool ASRModuleRos::buildASRGrammarXfei(){
  // 使用制定路径存放 grammar.bnf 文件（默认存放在 xbot_talker/cache/grammar_config/grammar 中）

  string grammar_build_path = grammar_path;
  string grammar_file = base_path + "/defaultconfig/grammar.bnf";
  // 讯飞语音sdk参数配置
  //这个路径前必须加一个fo|，否则就会语法构建不通过
  string asr_res_path = "fo|res/asr/common.jet";

  // 调用 build_grammar 函数，生成语法文件
  build_grammar_.buildGrammar(&asr_data_, base_path, grammar_build_path, asr_res_path, grammar_file);
  return true;

}
void ASRModuleRos::getParams(){
  // 获取参数
  asr_nodehandle.param("/asr_sample/enable_xfei_online", enable_xfei_online, bool(false));
  asr_nodehandle.param("/asr_sample/enable_baidu_online", enable_baidu_online, bool(false));
  asr_nodehandle.param("/asr_sample/enable_xfei_offline", enable_xfei_offline, bool(true));

  asr_nodehandle.param("/asr_sample/use_pcm_file", use_pcm_file, bool(false));
  asr_nodehandle.param("/asr_sample/use_mic", use_mic, bool(true));
  asr_nodehandle.param("/asr_sample/enable_record_save", enable_record_save, bool(true));

  asr_nodehandle.param("/asr_sample/base_path", base_path, std::string("~/catkin_ws/src/xbot_talker"));
  asr_nodehandle.param("/asr_sample/grammar_path", grammar_path,
                       std::string("~/catkin_ws/src/xbot_talker/default_config/grammar"));
  asr_nodehandle.param("/asr_sample/pcm_file", pcm_file,
                       std::string("~/catkin_ws/src/xbot_talker/defaultconfig/audio/nihao_test.pcm"));
  asr_nodehandle.param("/asr_sample/log_path", log_path, std::string("~/catkin_ws/src/xbot_talker/cache/log"));

  asr_nodehandle.param("/asr_sample/audio_channel", audio_channel, int(1));
  asr_nodehandle.param("/asr_sample/record_time", record_time, float(5));
  asr_nodehandle.param("/asr_sample/baidu_api_key", baidu_api_key_, std::string("kVcnfD9iW2XVZSMaLMrtLYIz"));
  asr_nodehandle.param("/asr_sample/baidu_secret_key", baidu_secret_key_, std::string("O9o1O213UgG5LFn0bDGNtoRN3VWl2du6"));


}


// 发布的话题
void ASRModuleRos::advertiseTopics()
{
  // 综合完整语音识别流程发布的topic
  recog_result_pub = asr_nodehandle.advertise<xbot_talker::recog_result>("/talker/offline_recog_result", 100);
  online_result_pub = asr_nodehandle.advertise<xbot_talker::online_asr_result>("/talker/online_recog_result", 100);

  // 各个模块单独的topic
  baidu_online_recog_result_ = asr_nodehandle.advertise<xbot_talker::online_asr_result>("/talker/baidu_online_recog_result", 100);
  xfei_online_recog_result_ = asr_nodehandle.advertise<xbot_talker::online_asr_result>("/talker/xfei_online_recog_result", 100);
  xfei_offline_recog_result_ = asr_nodehandle.advertise<xbot_talker::recog_result>("/talker/xfei_offline_recog_result", 100);
  enable_chat_pub_ = asr_nodehandle.advertise<std_msgs::Bool>("/nlp_module/enable_chat", 100);
}

// 订阅的话题
void ASRModuleRos::subscribeTopics()
{
  enable_asr_sub_ = asr_nodehandle.subscribe(std::string("/talker/enable_asr"), 10,
                                             &ASRModuleRos::subscribeEnableASR, this);
  enable_beep_sub_ = asr_nodehandle.subscribe(std::string("/asr_module/enable_beep"), 10,
                                             &ASRModuleRos::subscribeEnableBeep, this);
}

void ASRModuleRos::advertiseService()
{
  chat_server_ = asr_nodehandle.advertiseService(std::string("/xbot/chat"),&ASRModuleRos::requestChat, this);
  talker_chat_server_ = asr_nodehandle.advertiseService(std::string("/talker/chat"),&ASRModuleRos::requestTalkerChat, this);
  get_version_server_ = asr_nodehandle.advertiseService(std::string("/talker/version"),&ASRModuleRos::requestVersion, this);
  keyword_config_service = asr_nodehandle.advertiseService(std::string("/xbot_talker/asr_keyword_config"),&ASRModuleRos::requestKeywordConfig, this);
}

bool ASRModuleRos::requestVersion(xbot_talker::CallVersion::Request &req, xbot_talker::CallVersion::Response &res){
  res.version = "3.2.4";
  res.description = "xbot talker ";
  return true;
}

bool ASRModuleRos::requestChat(xbot_talker::chat::Request &req, xbot_talker::chat::Response &res){
  if(req.start_chat){
    enable_beep_ = true;
    std_msgs::Bool msg;
    msg.data =true;
    enable_chat_pub_.publish(msg);
    subscribeEnableASR(msg);
    res.chat_success =true;
    return true;
  }
  return true;
}

bool ASRModuleRos::requestTalkerChat(xbot_talker::CallChat::Request &req, xbot_talker::CallChat::Response &res){
  if(req.start_chat){
    enable_beep_ = true;
    std_msgs::Bool msg;
    msg.data =true;
    enable_chat_pub_.publish(msg);
    subscribeEnableASR(msg);
    res.chat_success =true;
    return true;
  }
  return true;
}

void ASRModuleRos::subscribeEnableBeep(const std_msgs::Bool msg){
  enable_beep_ = msg.data;

}

void ASRModuleRos::subscribeEnableASR(const std_msgs::Bool msg){
  if (msg.data == true)
  {
    if(enable_beep_){
      //播放“嘟”声提示音.
      std::string dialogue_prompt_wav = "play " + base_path + "/defaultconfig/audio/call.wav";
      system(dialogue_prompt_wav.c_str());
      //tts_play_.play_wav(base_path + "/defaultconfig/audio/call.wav");
    }
    offline_recog_result = "";
    baidu_online_result = "";
    xfei_online_result = "";
    ROS_INFO_STREAM("--------Speech recognition is awakened!--------");
    std::thread onlineThreadBaidu(&ASRModuleRos::onlineRecogBaidu, this);
    std::thread offlineThread(&ASRModuleRos::offlineRecog, this);

    offlineThread.join();
    onlineThreadBaidu.join();
    // 只要开启了讯飞离线模式，那么识别结果就以离线识别为准，只有在识别结果不够准确时，才根据是否开启在线语音识别发布结果
    if (enable_xfei_offline){
      if(enable_xfei_online){
        onlineRecogXfei();
      }
      if (std::stoi(offline_result_vector[1]) >= 30)
      {
        // 科大讯飞离线识别结果准确度>=35时，默认为识别准确，直接发布离线识别结果，不再进行在线处理。
        pubOfflineRecogResult(offline_result_vector[0], std::stoi(offline_result_vector[1]));
      }
      else if (enable_xfei_online == true && offline_result_vector[0] != "none_result")
      {
        // 如果离线准确度小于35，且离线结果不为空（离线结果为空意味着录音时间段内没有任何语音输入，十分安静），
        // 并且讯飞在线识别功能开启的情况下，调用在线语音进行识别，发布在线识别结果。
        pubOnlineRecogResult(xfei_online_result);
      }
      else if (enable_baidu_online == true && offline_result_vector[0] != "none_result")
      {
        // 同上。同等网络状况下，百度在线识别比科大讯飞在线识别慢很多，甚至因网络状况出现卡顿。
        // 所以建议使用科大讯飞在线和离线结合的方式。
        pubOnlineRecogResult(baidu_online_result);
      }
      else
      {
        pubOfflineRecogResult(offline_result_vector[0], std::stoi(offline_result_vector[1]));
      }

    }else if(enable_baidu_online && !enable_xfei_offline && !enable_xfei_online){
      // 发布百度识别结果
      pubOnlineRecogResult(baidu_online_result);

    }else if(enable_xfei_online && !enable_baidu_online && !enable_xfei_offline){
      std::thread onlineThreadXfei(&ASRModuleRos::onlineRecogXfei, this);
      onlineThreadXfei.join();
      // 只进行科大讯飞在线语音识别并发布
      pubOnlineRecogResult(xfei_online_result);
    }
    ROS_INFO_STREAM("Baidu online speech recog result: " << baidu_online_result << std::endl);
    ROS_INFO_STREAM("Xunfei online speech recog result: " << xfei_online_result << std::endl);
    ROS_INFO_STREAM("Xunfei offline speech recog result: " << offline_result_vector[0] << std::endl);
    pcm_buff.data = NULL;
    pcm_buff.size = 0;
  }

}

// 添加新的关键词构建语法后，重新配置语法参数
bool ASRModuleRos::requestKeywordConfig(xbot_talker::keyword_config::Request& req,
                                        xbot_talker::keyword_config::Response& res)
{
  if (req.keyword != "")
  {
    std::string bnf_word;
    bnf_word = config_file.readFileAsString(base_path + "/defaultconfig/grammar.bnf");
    ROS_INFO_STREAM("BNF file :" << bnf_word);
    CommonConfig& xunfei_config = CommonConfig::get_instance();
    asr_params_ = xunfei_config.configGramParas(base_path, grammar_path);
    res.error_code = 0;
  }
  else
  {
    res.error_code = 1;
  }
}

// 识别结果的发布.
void ASRModuleRos::pubOfflineRecogResult(const std::string result, const int accuracy)
{
  xbot_talker::recog_result recog_result;
  recog_result.recog_result = result;
  recog_result.recog_accuracy = accuracy;
  recog_result_pub.publish(recog_result);
}

// 完整的百度在线语音识别结果的发布.
void ASRModuleRos::pubOnlineRecogResult(const std::string result)
{
  xbot_talker::online_asr_result recog_result;
  recog_result.online_asr_result = result;
  online_result_pub.publish(recog_result);
}

// 科大讯飞离线语音识别线程
void ASRModuleRos::offlineRecog()
{
  recog_result_json = NULL;
  if (enable_xfei_offline == true)
  {
    if (use_pcm_file == true)
    {
      xfei_sr_offline.getPcmFileData();
    }
    else if (use_mic == true)
    {
      // 固定时长录音
      pcm_buff = xfei_sr_offline.recordThroughMIC(record_time, enable_record_save);
      xfei_sr_offline.getPCMData(pcm_buff);
      xfei_sr_offline.stopRecordThroughMIC();
    }
    // 科大讯飞识别模块的初始化
    xfei_sr_offline.initAsr();
    // 将全部音频数据循环写入科大讯飞接口进行识别并获取完整的json识别结果
    recog_result_json = xfei_sr_offline.dataLoopRecog();

    if (recog_result_json == NULL)
    {
      offline_result_vector[0] = "none_result";
      offline_result_vector[1] = "0";
      csv_file.writeRowData(log_path + "/asr_recog_result_log.csv", offline_result_vector);
    }
    else
    {
      // 在识别结果不为空时，从json结果中解析出需要的结果和置信度
      offline_result_vector = xfei_sr_offline.resultFromJson();
      csv_file.writeRowData(log_path + "/asr_recog_result_log.csv", offline_result_vector);
    }
    xbot_talker::recog_result recog_result;
    recog_result.recog_result = offline_result_vector[0];
    recog_result.recog_accuracy = std::stoi(offline_result_vector[1]);
    xfei_offline_recog_result_.publish(recog_result);
    // 一次识别结束后释放资源
    xfei_sr_offline.uninitAsr();
  }
}

// 科大讯飞在线语音识别线程
// 科大讯飞在线识别和离线识别不能同时两个线程进行。
void ASRModuleRos::onlineRecogXfei()
{
  char* recog_result = NULL;
  if (enable_xfei_online == true)
  {
    if (use_pcm_file == true)
    {
      // 获取pcm_file的数据
      xfei_sr_online.getPcmFileData();
    }
    else if (use_mic == true && !enable_xfei_offline)
    {
      struct DataBuff mic_pcm_buff = { NULL, 0 };
      ROS_INFO_STREAM("PCM" << pcm_buff.size << std::endl);
      mic_pcm_buff = xfei_sr_online.recordThroughMIC(record_time, enable_record_save);
      xfei_sr_online.getPCMData(mic_pcm_buff);
    }else if(use_mic == true && enable_xfei_offline){
      xfei_sr_online.getPCMData(pcm_buff);
    }
    // 科大讯飞识别模块的初始化
    xfei_sr_online.initAsr();
    // 将全部音频数据循环写入科大讯飞接口进行识别并获取完整的json识别结果
    recog_result = xfei_sr_online.dataLoopRecog();
    // 有时会出现离线识别结果置信度很低，但实际上无语音输入的情况，此情况下在线识别结果为空，需要加一个判断
    if (recog_result == NULL)
    {
      xfei_online_result = "none_result";
    }
    else
    {
      xfei_online_result = recog_result;
    }
    xfei_sr_online.uninitAsr();


    ROS_INFO_STREAM("Xunfei online iat result is :" << xfei_online_result << std::endl);

    std::ofstream result_log(log_path + "/asr_online_recog_result_log.txt", std::ios::app);
    if (result_log.is_open())
    {
      result_log << online_log_count_ << ":" << xfei_online_result << std::endl;
      online_log_count_++;
    }
    else
    {
      ROS_ERROR_STREAM("LOG ERROR");
    }
    result_log.close();

    xbot_talker::online_asr_result recog_result;
    recog_result.online_asr_result = xfei_online_result;
    xfei_online_recog_result_.publish(recog_result);
  }
}
// 百度在线语音识别线程
void ASRModuleRos::onlineRecogBaidu()
{
  if (enable_baidu_online == true)
  {
    if (use_pcm_file == true)
    {  // 读取pcm文件里的音频数据
      baidu_sr_online.getPcmFileData();
    }
    else if (use_mic == true)
    {
      struct DataBuff pcm_buff = { NULL, 0 };
      // 录音
      pcm_buff = baidu_sr_online.recordThroughMIC(record_time, enable_record_save);
      baidu_sr_online.getPCMData(pcm_buff);
    }
    char* recog_result_online;
    // 百度在线语音识别并获取识别结果
    recog_result_online = baidu_sr_online.runAsrAndRecog();
    // 从完整的json语音识别结果中解析出需要的字符串结果
    baidu_online_result = baidu_sr_online.resultFromJson();

    if (baidu_online_result.empty())
    {
      baidu_online_result = "none_result";
    }
    xbot_talker::online_asr_result recog_result;
    recog_result.online_asr_result = baidu_online_result;
    baidu_online_recog_result_.publish(recog_result);
    // 一次识别结束后释放资源
    baidu_sr_online.uninitAsr();

    std::ofstream result_log(log_path + "/asr_online_recog_result_log.txt", std::ios::app);
    if (result_log.is_open())
    {
      result_log << online_log_count_ << ":" << recog_result_online << std::endl;
      online_log_count_++;
    }
    else
    {
      ROS_ERROR_STREAM("LOG ERROR");
    }
    result_log.close();
    ROS_INFO_STREAM("Online asr result is :" << recog_result_online << std::endl);
    ROS_INFO_STREAM("Online asr finall result is :" << baidu_online_result << std::endl);
  }
}





/**************************************************
*************   Ros NLP Module  *******************
***************************************************/
NLPModuleRos::NLPModuleRos():answer_dictionary_(4)
{
  enable_chat_ =false;
}
NLPModuleRos::~NLPModuleRos()
{
}
bool NLPModuleRos::init()
{
  advertiseTopics();
  subscribeTopics();
  advertiseService();
  // 获取参数
  nlp_nodehandle.param("/nlp_sample/base_path", base_path, std::string("~/catkin_ws/src/xbot_talker"));
  nlp_nodehandle.param("/nlp_sample/nlp_config_path", nlp_config_path,
                       std::string("~/catkin_ws/src/xbot_talker/defaultconfig/answer_dic.csv"));

  nlp_nodehandle.param("/nlp_sample/log_path", log_path, std::string("~/catkin_ws/src/xbot_talker/cache/log"));
  nlp_nodehandle.param("/nlp_sample/tuling_key", tuling_key_, std::string("b4aac2a556b042269e418c78aa88f4cc"));

  // 科大讯飞登录
  CommonConfig& xunfei_config = CommonConfig::get_instance();
  xunfei_config.loginToXunfei(base_path);

  // 如果唤醒节点存在，启动nlp时会自动使能唤醒
  for (int i = 0; i < 2; i++)
  {
    pubStartAwaken(true);
    sleep(1);
  }
  ROS_INFO_STREAM("--------Result feedback module is ready to start!--------");

  // 读取对话配置文件内容保存进answer_table变量
  result_handle.readAnswerTable(nlp_config_path);
}

void NLPModuleRos::advertiseTopics()
{
  start_asr_pub_ = nlp_nodehandle.advertise<std_msgs::Bool>("/talker/enable_asr", 100);
  start_awaken_pub = nlp_nodehandle.advertise<std_msgs::Bool>("/talker/enable_awake", 100);
  mov_control_pub_ = nlp_nodehandle.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);

  pitch_platform_pub_ = nlp_nodehandle.advertise<std_msgs::Int8>("/mobile_base/commands/pitch_platform", 100);
  yaw_platform_pub_ = nlp_nodehandle.advertise<std_msgs::Int8>("/mobile_base/commands/yaw_platform", 100);

  left_get_pub = nlp_nodehandle.advertise<std_msgs::Empty>("/left_arm/commands/lift_up", 10);
  right_get_pub = nlp_nodehandle.advertise<std_msgs::Empty>("/right_arm/commands/lift_up", 10);

  left_put_pub = nlp_nodehandle.advertise<std_msgs::Empty>("/left_arm/commands/put_down", 10);
  right_put_pub = nlp_nodehandle.advertise<std_msgs::Empty>("/right_arm/commands/put_down", 10);

  left_grip_pub = nlp_nodehandle.advertise<std_msgs::Bool>("/left_arm/commands/grip", 10);
  right_grip_pub = nlp_nodehandle.advertise<std_msgs::Bool>("/right_arm/commands/grip", 10);
  reset_arm_pub_ = nlp_nodehandle.advertise<std_msgs::Empty>("/arm/commands/reset", 10);
  arm_lift_up_pub_ = nlp_nodehandle.advertise<std_msgs::Empty>("/arm/commands/lift_up", 10);
  arm_put_down_pub_ = nlp_nodehandle.advertise<std_msgs::Empty>("/arm/commands/put_down", 10);

  gripper_control_pub_ = nlp_nodehandle.advertise<std_msgs::Bool>("/arm/commands/grip", 10);

  // 发布导航相关的话题
  navi_start_pub_ = nlp_nodehandle.advertise<std_msgs::Bool>("/demo/visit", 10);
  navi_canel_pub_ = nlp_nodehandle.advertise<std_msgs::Bool>("/demo/navi_pause", 10);
  navi_continue_pub_ = nlp_nodehandle.advertise<std_msgs::Bool>("/demo/navi_continue", 10);
  navi_to_pose_pub_ = nlp_nodehandle.advertise<std_msgs::String>("/demo/navi_to_pose", 10);

  welcome_yes_pub = nlp_nodehandle.advertise<std_msgs::Bool>("/welcome/yes", 10);
  welcome_kp_pub = nlp_nodehandle.advertise<std_msgs::Bool>("/demo/leave", 10);

  enable_beep_pub_ = nlp_nodehandle.advertise<std_msgs::Bool>("/asr_module/enable_beep", 10);
}

void NLPModuleRos::subscribeTopics()
{
  recog_result_sub = nlp_nodehandle.subscribe(std::string("/talker/offline_recog_result"), 10,
                                              &NLPModuleRos::subscribeOfflineRecogResult, this);
  online_result_sub = nlp_nodehandle.subscribe(std::string("/talker/online_recog_result"), 10,
                                               &NLPModuleRos::subscribeOnlineRecogResult, this);
  enable_chat_sub_ = nlp_nodehandle.subscribe(std::string("/nlp_module/enable_chat"), 10,
                                              &NLPModuleRos::subscribeEnableChat, this);
}

void NLPModuleRos::advertiseService()
{
  dialog_config_service = nlp_nodehandle.advertiseService(std::string("/xbot_talker/nlp_dialog_config"),
                                                          &NLPModuleRos::requestDialogConfig, this);
  call_tts_ = nlp_nodehandle.serviceClient<xbot_talker::XbotTts>("/talker/xbot_tts");
}

void NLPModuleRos::subscribeEnableChat(const std_msgs::Bool msg){
  enable_chat_ = msg.data;
}

bool NLPModuleRos::callTTServer(std::string txt){
  xbot_talker::XbotTts tts;
  tts.request.start_tts = true;
  tts.request.tts_text = txt;
  if(call_tts_.call(tts)){
    if(tts.response.success){
      return true;
    }
  }else{
    ROS_ERROR("Failed to call /talker/xbot_tts service");
    return false;
  }

}

bool NLPModuleRos::requestDialogConfig(xbot_talker::nlpdialog_config::Request& req,
                                       xbot_talker::nlpdialog_config::Response& res)
{
  if (req.keyword != "back_config")
  {
    bool is_repeate = result_handle.isKeywordRepeated(req.keyword);
    if (is_repeate == false)
    {
      CSVOperation csv_file;
      std::vector<std::string> dialog_add;
      dialog_add.push_back(req.keyword);
      dialog_add.push_back(req.answer);
      dialog_add.push_back(std::to_string(req.action));
      dialog_add.push_back(std::to_string(req.action_mode));
      csv_file.writeRowData(nlp_config_path, dialog_add);
      // 重新读取对话配置文件内容保存进answer_table变量
      result_handle.readAnswerTable(nlp_config_path);
      res.error_code = 0;
      return true;
    }
    else
    {
      res.error_code = 1;
      return false;
    }
  }
  else
  {
    // 重新读取对话配置文件内容保存进answer_table变量
    result_handle.readAnswerTable(nlp_config_path);
    return true;
  }
}

void NLPModuleRos::pubStartRecog( )
{
  std_msgs::Bool msg;
  msg.data =true;
  start_asr_pub_.publish(msg);
}

void NLPModuleRos::pubStartAwaken(bool enable_awake)
{
  std_msgs::Bool enable_awaken;
  enable_awaken.data = enable_awake;
  start_awaken_pub.publish(enable_awaken);
}
// 控制导航相关动作
void NLPModuleRos::pubNaviControl(const int robot_action)
{
  enable_chat_ = false;
  std_msgs::String  string_msg;
  std_msgs::Bool bool_msg;
  switch (robot_action)
  {
    case NAVI_START:
      bool_msg.data = true;
      navi_start_pub_.publish(bool_msg);
      usleep(10000);
      welcome_kp_pub.publish(bool_msg);
      usleep(10000);
      welcome_yes_pub.publish(bool_msg);
      break;
    case NAVI_TO_POSE:
      string_msg.data = answer_dictionary_[1];
      navi_to_pose_pub_.publish(string_msg);
      break;
    case NAVI_CANCEL:
      bool_msg.data = true;
      navi_canel_pub_.publish(bool_msg);
      break;
    case NAVI_CONTINUE:
      bool_msg.data = true;
      navi_continue_pub_.publish(bool_msg);
      break;
    case BACK_TO_ORIGINAL_LOCATION:
      string_msg.data = "Starting_Point";
      navi_to_pose_pub_.publish(string_msg);
      break;
    case NAVI_TO_FIRST_POSE:
      string_msg.data = "First_Goal";
      navi_to_pose_pub_.publish(string_msg);
      break;
    case NAVI_TO_SECOND_POSE:
      string_msg.data = "Second_Goal";
      navi_to_pose_pub_.publish(string_msg);
      break;
    case NAVI_TO_THIRD_POSE:
      string_msg.data = "Third_Goal";
      navi_to_pose_pub_.publish(string_msg);
      break;
    case NAVI_TO_FOURTH_POSE:
      string_msg.data = "Fourth_Goal";
      navi_to_pose_pub_.publish(string_msg);
      break;
    case NAVI_TO_FIFTH_POSE:
      string_msg.data = "Fifth_Goal";
      navi_to_pose_pub_.publish(string_msg);
    break;
  }
}


// 控制机械臂动作
void NLPModuleRos::pubArmControl(const int robot_action)
{
  std_msgs::Empty empty_msg;
  std_msgs::Bool bool_msg;
  switch (robot_action)
  {
    case RESET_ARM:
      reset_arm_pub_.publish(empty_msg);
      sleep(7);
      break;
    case RAISE_LEFT_HAND:
      left_get_pub.publish(empty_msg);
      sleep(7);
      break;
    case PUT_DOWM_LEFT_HAND:
      left_put_pub.publish(empty_msg);
      sleep(7);
      break;
    case RAISE_RIGHT_HAND:
      right_get_pub.publish(empty_msg);
      sleep(7);
      break;
    case PUT_DOWM_RIGHT_HAND:
      right_put_pub.publish(empty_msg);
      sleep(7);
      break;
    case LEFT_HAND_GRIP:
      bool_msg.data = true;
      left_grip_pub.publish(bool_msg);
      sleep(1);
      break;
    case LEFT_HAND_OPEN:
      bool_msg.data = false;
      left_grip_pub.publish(bool_msg);
      sleep(1);
      break;
    case RIGHT_HAND_GRIP:
      bool_msg.data = true;
      right_grip_pub.publish(bool_msg);
      sleep(1);
      break;
    case RIGHT_HAND_OPEN:
      bool_msg.data = false;
      right_grip_pub.publish(bool_msg);
      sleep(1);
      break;
    case RAISE_ARM:
      arm_lift_up_pub_.publish(empty_msg);
      sleep(7);
      break;
    case PUT_DOWN_ARM:
      arm_put_down_pub_.publish(empty_msg);
      sleep(7);
      break;
    case OPEN_GRIPPER:
      bool_msg.data = false;
      gripper_control_pub_.publish(bool_msg);
      sleep(1);
      break;
    case CLOSE_GRIPPER:
      bool_msg.data = true;
      gripper_control_pub_.publish(bool_msg);
      sleep(1);
      break;
  }
}


// 控制机器人进行前进、后退、左转、右转、头部平台转动等动作
void NLPModuleRos::pubMoveControl(const int robot_action)
{
  geometry_msgs::Twist twist;
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;

  std_msgs::Int8 int_msg;

  switch (robot_action)
  {
    case TAKE_A_STEP_FORWARD:
      twist.linear.x = 0.15;
      mov_control_pub_.publish(twist);
      break;

    case TAKE_A_STEP_BACKWARD:
      twist.linear.x = -0.15;
      mov_control_pub_.publish(twist);

      break;
    case TURN_LEFT:
      twist.angular.z = 0.78;
      mov_control_pub_.publish(twist);
      break;

    case TURN_RIGHT:
      twist.angular.z = -0.78;
      mov_control_pub_.publish(twist);
      break;
    case LOOK_UP:
      int_msg.data = 40;
      pitch_platform_pub_.publish(int_msg);
      break;
    case LOOK_DOWN:
      int_msg.data = -40;
      pitch_platform_pub_.publish(int_msg);
      break;
    case LOOK_LEFT:
      int_msg.data = 60;
      yaw_platform_pub_.publish(int_msg);
      break;
    case LOOK_RIGHT:
      int_msg.data = -60;
      yaw_platform_pub_.publish(int_msg);
      break;
    case LOOK_FORWARD:
      int_msg.data = 0;
      pitch_platform_pub_.publish(int_msg);
      usleep(100000);
      yaw_platform_pub_.publish(int_msg);
      break;
  }
}

void NLPModuleRos::subscribeOfflineRecogResult(const xbot_talker::recog_result msg)
{
  ROS_INFO_STREAM("recog_result is :" << msg.recog_result << " |recog_accuracy is :" << msg.recog_accuracy
                                      << std::endl);

  std::ofstream result_log(log_path + "/recog_result_log.txt", std::ios::app);
  if (result_log.is_open())
  {
    result_log << log_count_ << ":" << msg.recog_result << std::endl;
    log_count_++;
  }
  else
  {
    ROS_ERROR_STREAM("LOG ERROR");
  }
  result_log.close();
  // 判断识别结果是否有效.
  bool ret = result_handle.resultIsValid(msg.recog_result);
  if (ret == false)
  {
    // 无效的识别结果意味着录音时间段内没有人说话，播放“没有人说话吗？那我先退下了”提示音并退出对话。
    // 若唤醒节点开启，则会进入唤醒模式
    if(enable_chat_){
      tts_play_.play_wav(base_path + "/defaultconfig/audio/non_talker.wav");
      pubStartAwaken(true);
      pubStartRecog();
    }else{
      tts_play_.play_wav(base_path + "/defaultconfig/audio/none_speech.wav");
      pubStartAwaken(true);
    }
  }
  else if (msg.recog_accuracy < 10)
  {
    // 离线识别准确度小于10,可认定有人说话但说的与定义好的语法关键词没有任何关系
    // 播放“你的问题我还不会回答呢，请试试别的问题吧”提示音，并进入语音识别模式
    std::string sorry_prompt_wav = "play " + base_path + "/defaultconfig/audio/donot_know.wav";
    //system(sorry_prompt_wav.c_str());
    tts_play_.play_wav(base_path + "/defaultconfig/audio/donot_know.wav");
    pubStartRecog();
  }
  else if (msg.recog_accuracy < 30)
  {
    // 离线识别准确度在10-35之间，可认为用户说了语法关键词，但因为噪声等原因导致准确度很低，为避免出现误操作，需要提示用户确认一遍
    // 播放“识别不够准确，请再说一遍命令词吧”提示音，并进入语音识别模式
    std::string sorry_prompt_wav = "play " + base_path + "/defaultconfig/audio/low_SC.wav";
    //system(sorry_prompt_wav.c_str());
    tts_play_.play_wav(base_path + "/defaultconfig/audio/low_SC.wav");
    pubStartRecog();
  }
  else
  {
    // 根据定义的应答策略检测识别到的关键词的识别策略.
    answer_dictionary_ = result_handle.keywordDetection();
    robotResponse();
  }
  result_handle.uninitNLP();
}

bool NLPModuleRos::robotAction(){
  // 监测是否需要机器人控制以及控制类型
  int robot_action = stoi(answer_dictionary_[2]);

  std_msgs::Bool bool_msg;

  if (robot_action == START_CHAT)
  {
    enable_chat_ = true;
    bool_msg.data =true;
    enable_beep_pub_.publish(bool_msg);
    usleep(100000);
    return true;
  }else if (robot_action == CLOSE_CHAT){
    ROS_INFO_STREAM("-------- The conversation is closed! --------");
    bool_msg.data =false;
    enable_beep_pub_.publish(bool_msg);
    enable_chat_ = false;
    return true;
  }else if(robot_action == NO_ACTION){
    return true;
  }
  else if ((robot_action >100) && (robot_action <200))
  {
    pubMoveControl(robot_action);
    sleep(1);
  }

  else if ((robot_action >200) && (robot_action <300))
  {
    pubArmControl(robot_action);
    sleep(1);
  }

  else if ((robot_action >300) && (robot_action <400))
  {
    pubNaviControl(robot_action);
    enable_chat_ = false;
    bool_msg.data =false;
    enable_beep_pub_.publish(bool_msg);
    sleep(1);
  }else{
    ROS_ERROR_STREAM("Get wrong action number : "<<robot_action << " .Please check the answer_dictionary config!");
    return false;
  }

}


bool NLPModuleRos::robotResponse(){
  // 先判断是否有对应响应，若没有，播放语音提示并退出对话，若存在唤醒，就进入唤醒模式
  if(answer_dictionary_[0] == ""){
    std::string sorry_prompt_wav = "play " + base_path + "/defaultconfig/audio/check_answer_config.wav";
    //system(sorry_prompt_wav.c_str());
    tts_play_.play_wav(base_path + "/defaultconfig/audio/check_answer_config.wav");
    pubStartAwaken(true);
    return true;
  }

  // 先检查对应的响应模式，即对话表格第四列：只播放，只动作，先动作后播放等等……
  int action_mode = std::stoi(answer_dictionary_[3]);
  int robot_action = stoi(answer_dictionary_[2]);
  std::cout << "Action mode is:  " << action_mode << std::endl;

  switch (action_mode)
  {
    case PLAY_ONLY:
      if(callTTServer(answer_dictionary_[1])){
        break;
      }
      break;
    case ACTION_ONLY:
      if(robotAction()){
        break;
      }
      break;
    case PLAY_BEFORE_ACTION:
      if(callTTServer(answer_dictionary_[1])){
        if(robotAction()){
          break;
        }
      }
      break;
    case PLAY_AFTER_ACTION:
      if(robotAction()){
        sleep(2);
        callTTServer(answer_dictionary_[1]);
      }
      break;
    case PLAY_AND_ACTION_SYNC:
      robotAction();
      callTTServer(answer_dictionary_[1]);
      break;
    default:
      std::cout << "Error occurred in action_mode: " << action_mode << std::endl;
      break;
  }
  if(enable_chat_){
    pubStartRecog();
  }else
  {
    pubStartAwaken(true);
  }
}

void NLPModuleRos::subscribeOnlineRecogResult(const xbot_talker::online_asr_result msg)
{
  tuling_answer_text_ = "";
  ROS_INFO_STREAM("online asr result is :" << msg.online_asr_result << std::endl);
  std::ofstream result_log(log_path + "/recog_result_log.txt", std::ios::app);
  if (result_log.is_open())
  {
    result_log << log_count_ << ":" << msg.online_asr_result << std::endl;
    log_count_++;
  }
  else
  {
    ROS_ERROR_STREAM("LOG ERROR");
  }
  result_log.close();

  // 判断识别结果是否有效.
  bool ret = result_handle.resultIsValid(msg.online_asr_result);
  if (ret == false)
  {
    // 无效的识别结果意味着录音时间段内没有人说话，播放“没有人说话吗？那我先退下了”提示音并退出对话。
    // 若唤醒节点开启，则会进入唤醒模式
    std::string sorry_prompt_wav = "play " + base_path + "/defaultconfig/audio/none_speech.wav";
    //system(sorry_prompt_wav.c_str());
    tts_play_.play_wav(base_path + "/defaultconfig/audio/none_speech.wav");
    enable_chat_ = false;
    std_msgs::Bool bool_msg;
    bool_msg.data =false;
    enable_beep_pub_.publish(bool_msg);
    pubStartAwaken(true);
  }
  else
  {
    // 设置图灵机器人对话请求参数
    tuling_robot.setAskJson(tuling_key_,msg.online_asr_result);
    // 请求图灵机器人接口，获取回答
    tuling_robot.callTulingApi();
    // 从图灵机器人的完整json回答中解析出最终需要的text回答
    tuling_answer_text_ = tuling_robot.textFromJson();
    ROS_INFO_STREAM("TuLing robot answer is:" << tuling_answer_text_ << std::endl);
    // 一次回答后释放资源
    tuling_robot.uninitTuling();
    result_handle.resultIsValid(tuling_answer_text_);

    // 根据定义的应答策略检测识别到的关键词的识别策略.
    answer_dictionary_ = result_handle.keywordDetection();

    if (answer_dictionary_[0] != "")
    {
      robotResponse();
    }
    else
    {
      TextToSpeech text_to_speech;
      bool ret;
      ret = text_to_speech.audioConverter(base_path + "/cache/wav", tuling_answer_text_.c_str());
      if (enable_chat_)
      {
        pubStartRecog();
      }
      else
      {
        pubStartAwaken(true);
      }
    }
  }
  result_handle.uninitNLP();
}

/**************************************************
*************   Ros Awaken Module  *******************
***************************************************/
AwakenModuleRos::AwakenModuleRos()
{
  // 加载ros param相关参数
  getParams();
  // 注册对外发布的topics
  advertiseTopics();
  // 注册订阅的topics
  subscribeTopics();
}

AwakenModuleRos::~AwakenModuleRos()
{
}

bool AwakenModuleRos::init()
{

  awaken_module.loginAndSetParams(appid_, base_path, pcm_file, audio_channel);
  ROS_INFO_STREAM("--------Voice wake up module is ready to be waken up!--------");
}
void AwakenModuleRos::getParams(){
  // 获取参数
  awaken_nodehandle.param("/awaken_sample/appid", appid_, std::string("5f85684b"));

  awaken_nodehandle.param("/awaken_sample/awaken_mode", awaken_mode, std::string("mic"));
  awaken_nodehandle.param("/awaken_sample/base_path", base_path, std::string("~/catkin_ws/src/xbot_talker"));
  awaken_nodehandle.param("/awaken_sample/pcm_file", pcm_file,
                          std::string("~/catkin_ws/src/xbot_talker/defaultconfig/audio/awaken.pcm"));
  awaken_nodehandle.param("/awaken_sample/enable_record_save", enable_record_save, bool(false));

  awaken_nodehandle.param("/awaken_sample/audio_save_path", audio_save_path,
                          std::string("~/catkin_ws/src/xbot_talker/cache/"));
  awaken_nodehandle.param("/awaken_sample/log_path", log_path, std::string("~/catkin_ws/src/xbot_talker/cache/"));

  awaken_nodehandle.param("/awaken_sample/audio_channel", audio_channel, int(1));
  awaken_nodehandle.param("/awaken_sample/record_time", record_time, int(600));
}

void AwakenModuleRos::advertiseTopics()
{
  is_awaken_pub_ = awaken_nodehandle.advertise<std_msgs::Bool>("/talker/awakened", 100);
  enable_asr_pub_ = awaken_nodehandle.advertise<std_msgs::Bool>("/talker/enable_asr", 100);
}

void AwakenModuleRos::subscribeTopics()
{
  start_awaken_sub_ = awaken_nodehandle.subscribe(std::string("/talker/enable_awake"), 10,
                                                 &AwakenModuleRos::subscribeStartAwaken, this);
}

void AwakenModuleRos::pubStartRecog(bool is_awaken)
{
  std_msgs::Bool msg;
  msg.data = is_awaken;
  enable_asr_pub_.publish(msg);
}

void AwakenModuleRos::isWaken()
{
  while (true)
  {
    if (awaken_module.is_awaken)
    {
      if (awaken_mode == "mic")
      {
        awaken_module.stopRecordThroughMIC();
      }
      if (enable_record_save == true)
      {
        awaken_module.saveRecordDataToFile();
      }
      // 播放提示音提醒唤醒成功
      std::string dialogue_prompt_wav = "play " + base_path + "/defaultconfig/audio/wozai.wav";
      system(dialogue_prompt_wav.c_str());
      std_msgs::Bool msg;
      msg.data = true;
      is_awaken_pub_.publish(msg);
      // 开启离线命令词识别
      pubStartRecog(true);
      awaken_module.is_awaken = false;
      break;
    }
    else
    {
      continue;
    }
  }
}

void AwakenModuleRos::subscribeStartAwaken(const std_msgs::BoolConstPtr msg)
{
  if (msg->data == true)
  {
    std::thread check(&AwakenModuleRos::isWaken, this);
    awaken_module.awakenInit();
    if (awaken_mode == "file")
    {
      awaken_module.getPcmFileData();
      awaken_module.dataLoopAwaken();
    }
    else
    {
      awaken_module.recordThroughMIC(record_time, enable_record_save);
      // TODO:可打断模式下，需要改为在此处关闭录音,取消下行注释即可
      // awaken_module.stopRecordThroughMIC();
    }
    check.join();
    awaken_module.uninitAsr();
  }
}

/**************************************************
*************   Ros TTS Module  *******************
***************************************************/

TTSModuleRos::TTSModuleRos()
{
  // 获取参数
  tts_nodehandle.param("/tts_sample/base_path", base_path, std::string("~/catkin_ws/src/xbot_talker"));
  tts_nodehandle.param("/tts_sample/log_path", log_path, std::string("~/catkin_ws/src/xbot_talker/cache/log"));
  advertiseServices();
}
TTSModuleRos::~TTSModuleRos()
{
}
bool TTSModuleRos::init()
{
  // 科大讯飞asr登录以及语法参数配置
  CommonConfig& xunfei_config = CommonConfig::get_instance();
  xunfei_config.loginToXunfei(base_path);

  ROS_INFO_STREAM("--------TextToSpeech module is ready to start.--------");
}

void TTSModuleRos::advertiseServices()
{
  tts_server = tts_nodehandle.advertiseService(std::string("/talker/xbot_tts"), &TTSModuleRos::ttsCallback, this);
  play_server_ = tts_nodehandle.advertiseService(std::string("/xbot/play"), &TTSModuleRos::playCallback, this);
  talker_play_server_ = tts_nodehandle.advertiseService(std::string("/talker/play"), &TTSModuleRos::talkerPlayCallback, this);
}

bool TTSModuleRos::talkerPlayCallback(xbot_talker::CallPlay::Request &req, xbot_talker::CallPlay::Response &res){
  TextToSpeech tts_test;

  for(int i =0;i<req.loop_times+1;i++){
    if (req.mode == 1){
      // mode == 1，播放指定路径的音频文件，若用户无输入，则播放默认保存的测试文件
      if(req.audio_path == ""){
        ROS_INFO("No setting file input ");
        std::string audiofile = base_path+"/defaultconfig/audio/default_tts_test.wav";
        std::string dialogue_prompt_wav = "play " + audiofile;
        system(dialogue_prompt_wav.c_str());
      }else{
        std::string audiofile = req.audio_path;
        std::string dialogue_prompt_wav = "play " + audiofile;
        system(dialogue_prompt_wav.c_str());
      }
    }
    if (req.mode == 0){
      ROS_INFO("----------start to request the tts service-----------");

      if(req.tts_text==""){
        string text = "语音合成测试，如需合成其他文字，请设置文字输入";
        tts_test.audioConverter(log_path, text.c_str());
        ROS_INFO("tts  done.");

      }else{
        string text = req.tts_text;
        tts_test.audioConverter(log_path, text.c_str());
        ROS_INFO("tts  done.");
      }

    }
  }
  res.success = true;
  return true;
}

bool TTSModuleRos::playCallback(xbot_talker::play::Request &req, xbot_talker::play::Response &res){
  TextToSpeech tts_test;

  if (req.mode == 1){
    // mode == 1，播放指定路径的音频文件，若用户无输入，则播放默认保存的测试文件
    if(req.audio_path == ""){
      ROS_INFO("No setting file input ");
      std::string audiofile = base_path+"/defaultconfig/audio/default_tts_test.wav";
      std::string dialogue_prompt_wav = "play " + audiofile;
      system(dialogue_prompt_wav.c_str());
    }else{
      std::string audiofile = req.audio_path;
      std::string dialogue_prompt_wav = "play " + audiofile;
      system(dialogue_prompt_wav.c_str());
    }
  }
  if (req.mode == 2){
    ROS_INFO("----------start to request the tts service-----------");
    if(req.tts_text==""){
      string text = "语音合成测试，如需合成其他文字，请设置文字输入";
      tts_test.audioConverter(log_path, text.c_str());
      ROS_INFO("tts  done.");

    }else{
      string text = req.tts_text;
      tts_test.audioConverter(log_path, text.c_str());
      ROS_INFO("tts  done.");
    }
  }
  res.success = true;
  return true;
}

// xbot_tts 服务的回调函数
bool TTSModuleRos::ttsCallback(xbot_talker::XbotTts::Request& req, xbot_talker::XbotTts::Response& res)
{
  if (req.start_tts)
  {
    ROS_INFO("----------start to request the tts service-----------");
    TextToSpeech tts_test;
    string text = req.tts_text;
    tts_test.audioConverter(log_path, text.c_str());
    ROS_INFO("tts  done.");
    res.success = true;
    return true;
  }
}
