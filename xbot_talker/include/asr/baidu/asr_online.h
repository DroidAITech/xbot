/**
 * @file    asr_online.h
 * @brief   百度语音识别模块接口头文件.
 * TODO: 还需要添加版权、版本等信息
 * @author  Xiaoyun Wang.
 */

#ifndef ASR_ONLINE_H_
#define ASR_ONLINE_H_
#include <curl/curl.h>
#include "common_config.h"
#include "file_operation.h"
#include "iostream"
#include "linuxrec.h"

/** 百度语音识别相关参数的配置 */
struct AsrConfig
{
  /** 填写网页上申请的appkey 如 $apiKey="g8eBUMSokVB1BHGmgxxxxxx" */
  std::string api_key;
  /** 填写网页上申请的APP SECRET 如$secretKey="94dc99566550d87f8fa8ece112xxxxx" */
  std::string secret_key;
  /** 音频格式 */
  std::string format;
  /** 音频采样率 */
  int rate;
  /** 不填写lan参数生效，都不填写，默认1537（普通话输入法模型）*/
  int dev_pid;
  /** 用户唯一标识，用来区分用户 */
  std::string cuid;
  /** 有此scope表示有asr能力，没有请在网页里勾选，非常旧的应用可能没有 */
  std::string scope;
  /** 百度在线语音识别URL:http://vop.baidu.com/server_api */
  std::string url;
};

/** 百度在线语音识别模块 */
class BaiduAsrOnline
{
public:
  /**
   * @fn setAsrParams
   * @brief	设置语音模块需外部传入的路径等参数.
   * @param base_path			- [in] 到/xbot_talker目录的结对路径.
   * @param pcm_file      - [in] 用于进行语音识别的录音文件.
   * @param channel       - [in] 音频声道数.1表示单声道,2表示双声道.
   */
  void setAsrParams(std::string baidu_api_key, std::string baidu_srcret_key,const std::string base_path, const std::string pcm_file, const int channel);

  /**
   * @fn initAndConfigAsr
   * @brief	百度语音识别模块的初始化,设置asr相关参数.
   */
  void initAndConfigAsr();

  /**
   * @fn speechGetToken
   * @brief	获取百度在线语音识别的Token.
   */
  void speechGetToken();

  /**
   * @fn getPcmFileData
   * @brief 读取pcm文件里的音频数据,并将数据的内容和数据大小存入pcm_data_结构体.
   */
  void getPcmFileData();

  /**
   * @fn runAsrAndRecog
   * @brief	将pcm_data_结构体里存储的音频用于百度在线语音识别并获取识别结果.
   * @return	char*	rec_result_		- 返回在线语音识别的完整json结果.
   */
  char* runAsrAndRecog();

  /**
   * @fn resultFromJson
   * @brief 从完整的json语音识别结果中解析出需要的字符串结果.
   * @return std::string online_final_result  - 返回解析出的字符串识别结果.
   */
  std::string resultFromJson();

  /**
   * @fn recordThroughMIC
   * @brief	录音接口.
   * @param record_time                   - [in] 录音时长(s).
   * @param enable_audio_save             - [in] 是否将录音保存到pcm文件.
   * @return	struct DataBuff	pcm_data_		- 返回录音的pcm数据和大小.
   */
  struct DataBuff recordThroughMIC(const float record_time, bool enable_audio_save);

  /**
   * @fn getPCMData
   * @brief	获取pcm数据接口.
   * @param pcm_buff  - [in] pcm数据的内容和数据大小.
   */
  void getPCMData(struct DataBuff pcm_buff);

  /**
   * @fn stopRecordThroughMIC
   * @brief 关闭录音设备.
   */
  void stopRecordThroughMIC();

  /**
   * @fn uninitAsr
   * @brief 一次识别结束后释放资源.
   */
  void uninitAsr();

private:
  /**
   * @fn getOneChannelData
   * @brief 从双声道数据中分离出单声道数据.
   * @return struct DataBuff	pcm_data_		- 返回单声道的pcm数据和大小.
   */
  struct DataBuff getOneChannelData();

  /**
   * @fn saveRecordDataToFile
   * @brief 把录音数据存入/cache/pcm/目录下的pcm文件里，以次序和时间命名.
   */
  void saveRecordDataToFile();

  /**
   * @fn praseToken
   * @brief 从json中解析出token值.
   */
  void praseToken();

  /**
   * @fn writefunc
   * @brief  libcurl 返回回调函数.
   */
  static size_t writefunc(void* ptr, size_t size, size_t nmemb, char** result);

  struct DataBuff pcm_data_ = { NULL, 0 };  // 存储pcm音频数据
  char* rec_result_ = NULL;                 // 保存语音识别结果
  std::string online_final_result;
  std::string base_path_;
  std::string pcm_file_;
  int channel_;
  int record_loops_ = 0;
  std::string token_;
  char* url_response_ = NULL;
  const std::string API_TOKEN_URL = "http://openapi.baidu.com/oauth/2.0/token";
  struct recorder* asr_record_ = NULL;
  FileOperation pcm_file_operation;
  RecordAlsaAPI record_alsa;
  struct AsrConfig asr_config;
};

#endif
