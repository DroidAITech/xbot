#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <pthread.h>
#include <signal.h>
#include <errno.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include "AIUITest.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/document.h"
#include "Talker.h"
#include "speech_recognizer.h"
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#include "xbot_talker/play.h"
#include "xbot_talker/chat.h"

#include "xbot_talker/talk_monitor.h"

using namespace std;

#define FRAME_LEN 640
#define BUFFER_SIZE 4096
#define SAMPLE_RATE_44K     44100
#define SAMPLE_RATE_16K     16000
#define SAMPLE_RATE_8K      8000
#define MAX_GRAMMARID_LEN   32
#define MAX_PARAMS_LEN      1024
int n=0;

typedef struct _UserData {
  int     build_fini; //标识语法构建是否完成
  int     update_fini; //标识更新词典是否完成
  int     errcode; //记录语法构建或更新词典回调错误码
  char  grammar_id[MAX_GRAMMARID_LEN]; //保存语法构建返回的语法ID
}UserData;



void onPlayFinished(int code,string message);
void on_result(const char *result, char is_last);
void on_speech_begin();
void on_speech_end(int reason);
void signal_handler(int s);

void* offline_voice_recog_thread(void* session_begin_params);
int build_grm_cb(int ecode, const char *info, void *udata);
int build_grammar(UserData *udata);
const string parse_result_from_json(char*  jsonContent);


const char* movement_pub_topic = "/cmd_vel_mux/input/teleop";
string basePath ;

ros::Publisher mov_control_pub;
ros::Publisher talk_pub;
ros::Publisher leave_pub;
IAIUIAgent *agent ;
Talker talker;
ISpeechUtility* speechUtility;

char *g_result = NULL;
unsigned int g_buffersize = BUFFER_SIZE;
int ret;
bool isPlayingAudio;
string lastGoal;
bool isVerifying;

mutex mutex_chat , mutex_playing_audio,mutex_isfinished;
condition_variable condition_chat , condition_playing_audio,condition_isfinished;
bool isChatting;

//自定义的state
int play_mode;
pthread_t record_thread_id;

UserData asr_data;
string ASR_RES_PATH  ; //离线语法识别资源路径
string GRAMMAR_BUILD_PATH ; //构建离线语法识别网络生成数据保存路径
string GRAMMAR_FILE   ; //构建离线识别语法网络所用的语法文件
char asr_params[MAX_PARAMS_LEN];


bool request_chat(xbot_talker::chat::Request &req, xbot_talker::chat::Response &res);

bool request_play(xbot_talker::play::Request &req, xbot_talker::play::Response &res);

//bool request_talk_state(xbot_talker::talk_state::Request &req, xbot_talker::talk_state::Response &res);

int test;

int main(int argc,char** argv){

  //  初始化ROS节点
  ros::init(argc,argv,"xbot_talker");
  ros::NodeHandle nodeHandle;
  // 定义两个服务，请求播放和请求对话
  ros::ServiceServer service_chat = nodeHandle.advertiseService("/xbot/chat", request_chat);
  ros::ServiceServer service_play = nodeHandle.advertiseService("/xbot/play", request_play);
  test = 1;
  // 定义监控talker状态的话题。isChatting  isPlaying  play_mode
  talk_pub = nodeHandle.advertise<xbot_talker::talk_monitor>("/xbot/talk_state", 1000);

  mov_control_pub = nodeHandle.advertise<geometry_msgs::Twist>(movement_pub_topic,1000);
  leave_pub = nodeHandle.advertise<std_msgs::Bool>("/welcome/leave",1000);

  //    相对路径的根路径参数获取
  nodeHandle.param("/xbot_talker/base_path",basePath, string("/home/xbot/catkin_ws/src/xbot_talker"));
  //    在根路径的组合路径下找到配置文件并初始化talker
  ret = talker.init(basePath);
  if(ret==-1){
    cout<<"Talker init failed"<<endl;
    exit(0);
  }else{
    cout<<"Talker init success"<<endl;
  }
  //    讯飞语音sdk参数配置
  ASR_RES_PATH = "fo|res/asr/common.jet";  //这个路径前必须加一个fo|，否则就会语法构建不通过
  GRAMMAR_BUILD_PATH = "res/gramBuild";
  GRAMMAR_FILE = basePath+"/assets/grammar.bnf";//自定义语法文件


  //    讯飞sdk中注册的appid需要与下载的文件对应，否则报错
  string login_parameters = "appid = 5ade9569, work_dir = "+basePath+"/assets";
  ret = MSPLogin(NULL, NULL, login_parameters.c_str());
  if (MSP_SUCCESS != ret)	{
    cout<<"MSPLogin failed , Error code  "<<ret<<endl;
    exit(0);
  }

  memset(&asr_data, 0, sizeof(UserData));
  cout<<"Start building offline grammar for recognition ..."<<endl;
  //第一次使用某语法进行识别，需要先构建语法网络，获取语法ID，之后使用此语法进行识别，无需再次构建
  ret = build_grammar(&asr_data);
  if (MSP_SUCCESS != ret) {
    cout<<"Building grammar failed!"<<endl;
    exit(0);
  }
  while (1 != asr_data.build_fini){
    usleep(300 * 1000);
  }
  if (MSP_SUCCESS != asr_data.errcode){
    exit(0);
  }
  // 全局变量用于标识当前是否为对话状态
  isChatting = false;

  // 全局变量用于标识是否已经结束对话。
  //isFinished = false;

  //离线语法识别参数设置
  snprintf(asr_params, MAX_PARAMS_LEN - 1,
           "engine_type = local, asr_denoise=1,vad_bos=10000,vad_eos=10000,\
           asr_res_path = %s, sample_rate = %d, \
           grm_build_path = %s, local_grammar = %s, \
           result_type = json, result_encoding = UTF-8 ",
      ASR_RES_PATH.c_str(),
           SAMPLE_RATE_16K,
           GRAMMAR_BUILD_PATH.c_str(),
           asr_data.grammar_id
           );
  //    启动离线语音识别程序独立线程
  if(asr_params!=NULL){
    pthread_create(&record_thread_id,NULL,offline_voice_recog_thread,(void*)asr_params);
  }
  signal(SIGINT,signal_handler);
  ros::spin();
  return 0;
}

//bool request_talk_state(xbot_talker::talk_state::Request &req, xbot_talker::talk_state::Response &res)
//{
//  if (req.start_monitor == true)
//  {
//    ROS_INFO("request: %d", (int)req.start_monitor);
//    if (test == 1){
//      res.state = "is Chatting";
//    }
//    else
//    {
//      res.state="is not";
//    }

//  }
//}

//请求对话服务的回调函数
bool request_chat(xbot_talker::chat::Request &req, xbot_talker::chat::Response &res)
{
  if (req.start_chat == true)
  {
    n = 0;

    ROS_INFO("request: %d", (int)req.start_chat);
    //ROS_INFO("response: %d", (int)res.chat_success);
    onPlayFinished(REQUEST_CHAT, "chat");
    unique_lock<mutex> lock3(mutex_isfinished);
    condition_isfinished.wait(lock3);
    res.chat_success = true;
  }
}

//请求播放音频的回调函数
//TODO：加入循环播放
bool request_play(xbot_talker::play::Request &req,xbot_talker::play::Response &res)
{
  play_mode = req.mode;
  if(isPlayingAudio)
  {
    res.success = false;
    return false;
  }
  if (req.mode == 1)
  {

    string audiofile = req.audio_path;

    xbot_talker::talk_monitor talk_monitor;
    talk_monitor.isChatting = false;
    talk_monitor.isPlaying = true;
    talk_monitor.play_mode = 1;
    talk_pub.publish(talk_monitor);
    talker.play((char*)audiofile.c_str(),REQUEST_SIMPLE_PLAY,&onPlayFinished);

    talk_monitor.isChatting = false;
    talk_monitor.isPlaying = false;
    talk_monitor.play_mode = 0;
    talk_pub.publish(talk_monitor);
    res.success = true;

    return true;
  }
  if (req.mode == 2)
  {

    string text = req.tts_text;
    string file = "play_text.wav";

    xbot_talker::talk_monitor talk_monitor;
    talk_monitor.isChatting = false;
    talk_monitor.isPlaying = true;
    talk_monitor.play_mode = 2;
    talk_pub.publish(talk_monitor);
    talker.text_to_speech(text.c_str(),file.c_str(),REQUEST_SIMPLE_PLAY,onPlayFinished);
    talk_monitor.isChatting = false;
    talk_monitor.isPlaying = false;
    talk_monitor.play_mode = 0;
    talk_pub.publish(talk_monitor);
    res.success = true;

    return true;
  }


  res.success = false;
  return false;
}

//离线语音识别线程
void* offline_voice_recog_thread(void* session_begin_params){
  //    printf("sr_init param: %s",(char*)session_begin_params);
  int errcode;
  int i = 0;
  struct speech_rec iat;
  struct speech_rec_notifier recnotifier = {
        on_result,
        on_speech_begin,
        on_speech_end
  };
  while(true){
    //对话同步锁
    //问候完访客时 -- 开启该线程
    //对话超时或得到目标位置点  -- 挂起该线程
    unique_lock<mutex> lock1(mutex_chat);
    condition_chat.wait(lock1,[]{return isChatting;});

    signal(SIGINT,signal_handler);
    cout<<"-----------Start Chatting--------"<<endl;
    cout<<"You can speak to me : "<<endl;

    errcode = sr_init(&iat, (char*)session_begin_params, SR_MIC, &recnotifier);
    if (errcode) {
      cout<<"Speech recognizer init failed"<<endl;
      return NULL;
    }
    errcode = sr_start_listening(&iat, (char *)basePath.data());
    if (errcode) {
      cout<<"Start listening failed. code: "<< errcode<<endl;
    }
    //这里只睡眠4秒
    //由于讯飞sdk的原因，底层录音三秒就会停止录音，按照官方文档中设置了参数也没有用(貌似是 底层bug)
    //http://bbs.xfyun.cn/forum.php?mod=viewthread&tid=35056&extra=page%3D4
    sleep(4);
    //语音播放同步锁
    //即将开启下一轮录音，判断当前是否在播放语音
    //如果在播放语音，则挂起该线程，等待语音播放完
    if(isPlayingAudio){
      unique_lock<mutex> lock2(mutex_playing_audio);
      condition_playing_audio.wait(lock2,[]{return !isPlayingAudio;});
      errcode = sr_stop_listening(&iat);
      if (errcode) {
        cout<<"Stop listening failed. code:"<<errcode<<endl;
      }
    }else{
      errcode = sr_stop_listening(&iat);
      if (errcode) {
        cout<<"Stop listening failed. code:"<<errcode<<endl;
      }
    }
    cout<<"Recording completed"<<endl;

    sr_uninit(&iat);
  }
  //isFinished = true;
 // condition_isfinished.notify_one();
}



//Talker播放完成的回调函数
void onPlayFinished(int code,string message){
  //    cout<<"-----------------------  onPlayFinished  ---- tid:"<<this_thread::get_id()<<endl;
  //cout<<"code: "<<code<<endl;
  //signal(SIGINT,signal_handler);
  cout<<"OnPlayFinished -- message:"<<message<<endl;
  std_msgs::String mes;
  switch(code){
  case REQUEST_CHAT:
  {

    cout<<"REQUEST_CHAT"<<endl;
    isPlayingAudio = false;
    condition_playing_audio.notify_all();
    isChatting = true;
    condition_chat.notify_one();
    xbot_talker::talk_monitor talk_monitor;
    talk_monitor.isChatting = true;
    talk_monitor.isPlaying = false;
    talk_monitor.play_mode = 0;
    talk_pub.publish(talk_monitor);

   // void talker_state();
  }
    break;
  case REQUEST_AUDIO_UNMATCH:
  {
    if(message.length()>=2){
      break;
    }
    cout<<"REQUEST_AUDIO_UNMATCH  "<<message<<endl;
    std_msgs::UInt32 msg;
    //发送200表示进行人脸验证，验证当前xbot前方到底有没有人
    msg.data = 200;
    isChatting = false;
    isVerifying = true;

    isPlayingAudio = false;
    condition_playing_audio.notify_all();
  }
    break;
  case REQUEST_MOVE:
  {
    cout<<"REQUEST_MOVE :"<<message<<endl;
    isChatting = false;
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    if(message.find("move_go_forward")!=-1){
      twist.linear.x = 0.1;
      mov_control_pub.publish(twist);
      sleep(2);
      twist.linear.x = 0;
    }else if(message.find("move_go_back")!=-1){
      twist.linear.x = -0.1;
      mov_control_pub.publish(twist);
      sleep(2);
      twist.linear.x = 0;
    }else if(message.find("move_rotate_left")!=-1){
      twist.angular.z = 0.78;
      mov_control_pub.publish(twist);
      sleep(3);
      twist.angular.z = 0;
    }else if(message.find("move_rotate_right")!=-1){
      twist.angular.z = -0.78;
      mov_control_pub.publish(twist);
      sleep(3);
      twist.angular.z = 0;
    }else{
      cout<<"unknown message:"<<message<<endl;
    }
    mov_control_pub.publish(twist);
    isPlayingAudio = false;
    condition_playing_audio.notify_all();
    isChatting = true;
    condition_chat.notify_all();
  }
    break;
  case REQUEST_SIMPLE_PLAY:
  {
    cout<<"REQUEST_SIMPLE_PLAY"<<endl;
    isChatting = false;
  }
    break;
  case REQUEST_END_CHAT:
  {
    cout<<"REQUEST_END_CHAT"<<endl;
    isPlayingAudio = false;
    isChatting = false;
    xbot_talker::talk_monitor talk_monitor;
    talk_monitor.isChatting = false;
    talk_monitor.isPlaying = false;
    talk_monitor.play_mode = 0;
    talk_pub.publish(talk_monitor);
    condition_isfinished.notify_one();
  }
    break;
  case REQUEST_CHAT_UNMATCH:
  {
    if(message.length()>=2){
      break;
    }
    n++;
    cout<<"REQUEST_CHAT_UNMATCH  "<<message<<endl;
    isChatting = true;
    isPlayingAudio = false;
    condition_chat.notify_one();
  }
    break;
  case REQUEST_LEAVE:
  {
    std_msgs::Bool msg;
    msg.data = true;
    leave_pub.publish(msg);
    isPlayingAudio = false;
    isChatting = false;

  }
   default:
    break;
  }
}

//构建离线识别语法的回调函数
int build_grm_cb(int ecode, const char *info, void *udata){
  UserData *grm_data = (UserData *)udata;
  if (NULL != grm_data) {
    grm_data->build_fini = 1;
    grm_data->errcode = ecode;
  }

  if (MSP_SUCCESS == ecode && NULL != info) {
    cout<<"Build grammar success! Grammar ID:"<< info<<endl;
    if (NULL != grm_data){
      strcpy(grm_data->grammar_id,info);
    }
  }else{
    cout<<"Build grammar failed!  Error code:"<< ecode<<endl;
  }
  return 0;
}

//构建离线识别语法
int build_grammar(UserData *udata){
  FILE *grm_file = NULL;
  char *grm_content = NULL;
  unsigned int grm_cnt_len  = 0;
  char grm_build_params[MAX_PARAMS_LEN]  ;

  grm_file = fopen(GRAMMAR_FILE.c_str(), "rb");
  if(NULL == grm_file) {
    cout<<"Fail to open file: "<<GRAMMAR_FILE<<" -- "<<strerror(errno)<<endl;
    return -1;
  }

  fseek(grm_file, 0, SEEK_END);
  grm_cnt_len = ftell(grm_file);
  fseek(grm_file, 0, SEEK_SET);

  grm_content = (char *)malloc(grm_cnt_len + 1);
  if (NULL == grm_content)
  {
    cout<<"Alloc memory failed"<<endl;
    fclose(grm_file);
    grm_file = NULL;
    return -1;
  }
  fread((void*)grm_content, 1, grm_cnt_len, grm_file);
  grm_content[grm_cnt_len] = '\0';
  fclose(grm_file);
  grm_file = NULL;

  snprintf(grm_build_params, MAX_PARAMS_LEN - 1,
           "engine_type = local, \
           asr_res_path = %s, sample_rate = %d, \
           grm_build_path = %s, ",
           ASR_RES_PATH.c_str(),
           SAMPLE_RATE_16K,
           GRAMMAR_BUILD_PATH.c_str()
           );
  cout<<grm_build_params<<endl;
  ret = QISRBuildGrammar("bnf", grm_content, grm_cnt_len, grm_build_params, build_grm_cb, udata);

  free(grm_content);
  grm_content = NULL;

  return ret;
}

//语音识别的结果回调
void on_result(const char *result, char is_last){
  //    cout<<"on_result       ----  tid: "<<this_thread::get_id()<<endl;
  if(result){
    size_t left = g_buffersize - 1 - strlen(g_result);
    size_t size = strlen(result);
    if (left < size) {
      g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
      if (g_result)
        g_buffersize += BUFFER_SIZE;
      else {
        cout<<"Alloc memory failed\n"<<endl;
        return;
      }
    }

    strncat(g_result, result, size);

    if(is_last){
      isPlayingAudio = true;
      cout<<g_result<<endl;

      const string final_result = parse_result_from_json(g_result);
      cout<<"Speech result : "<<final_result.c_str()<<endl;
      talker.chat(final_result,onPlayFinished);

    }
  }
}

//语音识别开始
void on_speech_begin(){
  //    cout<<"on_speech_begin       ----  tid: "<<this_thread::get_id()<<endl;
  cout<<"Start listening........"<<endl;
  if (g_result){
    free(g_result);
  }
  g_result = (char*)malloc(BUFFER_SIZE);
  g_buffersize = BUFFER_SIZE;
  memset(g_result, 0, g_buffersize);

}

//语音识别结束
void on_speech_end(int reason){
  //    cout<<"on_speech_end     ----  thread id: "<<this_thread::get_id()<<endl;
  cout<<"on_speech_end :"<<reason<<endl;
  if (reason == END_REASON_VAD_DETECT){
    cout<<"Speaking done.  ";
  }

  if(reason==10114){
    cout<<"Request Timeout"<<endl;
  }

}

//离线语音识别时，从返回结果的json中解析出语音识别结果
const string parse_result_from_json(char*  jsonContent){
  rapidjson::Document doc ;
  doc.Parse(jsonContent);
  //by fw  string empty = "";
  string result = "";
  rapidjson::Value& vConfidence = doc["sc"];
  if(vConfidence.GetInt()<20){
    cout<<"conficence of recognized words is too low"<<endl;
    return result.c_str();
  }
  rapidjson::Value& wordArr = doc["ws"];
  for(int i=0;i<wordArr.Size();i++){
    rapidjson::Value& wordUnit = wordArr[i];
    string slotStr = wordUnit["slot"].GetString();
    if(slotStr.find("<content>")!=-1){
      rapidjson::Value& contentWord = wordUnit["cw"];
      rapidjson::Value& word = contentWord[0];
      result = word["w"].GetString();
      cout<<"result:"<<result<<"|confidence:"<<vConfidence.GetInt()<<endl;
      return result;
    }
    else if(slotStr.find("<place>")!=-1 || slotStr.find("<want>")!=-1){ 							//by fw  连通grammar.bnf也改了一点结构，理论上避免了类似于“我找一下向左转”语句的成立
      rapidjson::Value& contentWord = wordUnit["cw"];																//by fw
      rapidjson::Value& word = contentWord[0];																			//by fw
      result =  result + word["w"].GetString();																			//by fw
      cout<<"result:"<<result<<"|confidence:"<<vConfidence.GetInt()<<endl;          //by fw
    }																																									//by fw
  }
  return result;
}

void signal_handler(int s){
  ros::shutdown();
  MSPLogout();
  exit(0);
}


