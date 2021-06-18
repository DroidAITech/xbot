/**
 * @file    awaken_offline.h
 * @brief   科大讯飞离线唤醒模块接口定义头文件.
 * TODO: 还需要添加版权、版本等信息
 * @author  Xiaoyun Wang.
 */
#ifndef AWAKEN_OFFLINE_H_
#define AWAKEN_OFFLINE_H_
#include <iostream>
#include "file_operation.h"
#include "linuxrec.h"
/** 科大讯飞离线唤醒模块接 */
class AwakenOffline
{
public:
  /**
   * @fn loginAndSetParams
   * @brief	科大讯飞离线唤醒登录以及参数设置.
   * @param appid			- [in] 科大讯飞官网注册应用的appid.
   * @param base_path			- [in] 到/xbot_talker目录的结对路径.
   * @param pcm_file      - [in] 用于进行离线唤醒的录音文件.
   * @param channel       - [in] 音频声道数.1表示单声道,2表示双声道.
   */
  void loginAndSetParams(const std::string appid, const std::string base_path, const std::string pcm_file, const int channel);

  /**
   * @fn awakenInit
   * @brief	科大讯飞离线唤醒模块的初始化.
   */
  void awakenInit();

  /**
   * @fn uninitAsr
   * @brief 一次离线唤醒结束后释放资源.
   */
  void uninitAsr();

  /**
   * @fn getPcmFileData
   * @brief 读取pcm文件里的音频数据,并将数据的内容和数据大小存入pcm_data_结构体.
   */
  void getPcmFileData();

  /**
   * @fn writeAudioData
   * @brief	将pcm音频写入科大讯飞QIVWAudioWrite接口.
   * @param audio_data        - [in] pcm音频数据.
   * @param audio_len         - [in] pcm音频大小.
   */
  void writeAudioData(const char* audio_data, unsigned int audio_len);

  /**
   * @fn dataLoopAwaken
   * @brief	将音频数据循环写入科大讯飞接口进行离线唤醒.
   */
  void dataLoopAwaken();

  /**
   * @fn saveRecordDataToFile
   * @brief 把录音数据存入/cache/pcm/目录下的pcm文件里，以次序和时间命名.
   */
  void saveRecordDataToFile();

  /**
   * @fn recordThroughMIC
   * @brief	录音并进行语音唤醒.
   * @param record_time                    - [in] 录音时长(s).
   * @param enable_audio_save              - [in] 是否将录音保存到pcm文件.
   */
  void recordThroughMIC(const float record_time, bool enable_audio_save);

  /**
   * @fn stopRecordThroughMIC
   * @brief 关闭录音设备.
   */
  void stopRecordThroughMIC();

  /**
   * @fn checkIsAwaken
   * @brief 检测是否唤醒.
   */
  void checkIsAwaken();

  static char* awaken_result_;
  static bool is_awaken;

private:
  const int FRAME_LEN = 640;  // 16k采样率的16bit音频，一帧的大小为640B, 时长20ms
  const char* awaken_params_;
  struct DataBuff pcm_data_ = { NULL, 0 };  // 存储pcm音频数据
  std::string pcm_file_;                    // 保存的音频文件
  std::string base_path_;                   // 表示到xbot_talker的绝对路径
  const char* session_id_ = NULL;
  int audio_stat_;
  struct recorder* awaken_record_ = NULL;
  static int cb_ivw_msg_proc(const char* sessionID, int msg, int param1, int param2, const void* info, void* userData);
  FileOperation file_operation;
  RecordAlsaAPI record_alsa;
  int channel_;
  int record_loops_;
  struct DataBuff getOneChannelData();
};

#endif
