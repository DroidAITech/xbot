#include "awaken/awaken_offline.h"
#include <iostream>
#include <thread>
// awaken module
AwakenOffline awaken_demo;
// base_path表示到xbot_talker的绝对或者相对路径.
const std::string base_path = "../../..";
const int ONE_CHANNEL = 1;  // 单声道
const int TWO_CHANNEL = 2;  // 双声道测试只能用于读取已有pcm文件识别
const std::string appid = "5f85684b";


typedef enum AWAKEN_MODE {
  READ_PCM_FILE_TO_ASR = 0,
  RECORDE_TO_ASR,

} AWAKEN_MODE;

void usage(int& awaken_mode)
{
  std::cout << "---------  awaken demo  ------------" << std::endl;
  std::cout << "这是离线唤醒功能的示例demo.\n唤醒词：“小德小德”\n你可以选择通过已录好的pcm文件或者实时录音进行唤醒测试."
            << std::endl;
  std::cout << "0:选择已录好的pcm音频文件进行唤醒测试;" << std::endl;
  std::cout << "1:通过实时录音进行唤醒测试." << std::endl;
  std::cout << "请输入0或1选择不同方式:" << std::endl;
  std::cout << "awaken_mode :_\b";
  std::cin >> awaken_mode;
}

void isWaken_file()
{
  while (true)
  {
    if (awaken_demo.is_awaken)
    {
      // 播放提示音提醒唤醒成功
      std::string dialogue_prompt_wav = "play " + base_path + "/defaultconfig/audio/wozai.wav";
      system(dialogue_prompt_wav.c_str());
      // 唤醒后再切换至为唤醒状态等待下次唤醒
      awaken_demo.is_awaken = false;
      break;
    }
    else
    {
      continue;
    }
  }
}

void isWaken_mic()
{
  while (true)
  {
    if (awaken_demo.is_awaken)
    {  // 检测到唤醒后停止录音
      awaken_demo.stopRecordThroughMIC();
      // 将录音文件保存到xbot_talker/cache/pcm/目录
      awaken_demo.saveRecordDataToFile();
      // 播放提示音提醒唤醒成功
      std::string dialogue_prompt_wav = "play " + base_path + "/defaultconfig/audio/wozai.wav";
      system(dialogue_prompt_wav.c_str());
      // 唤醒后再切换至为唤醒状态等待下次唤醒
      awaken_demo.is_awaken = false;
      break;
    }
    else
    {
      continue;
    }
  }
}

int main(int argc, char** argv)
{
  int awaken_mode;
  usage(awaken_mode);
  // pcm_file表示用于用于作为输入的音频文件,默认为xbot_talker/defaultconfig/audio/awaken.pcm.
  std::string pcm_file = base_path + "/defaultconfig/audio/awaken.pcm";
  // channel 音频声道数.1表示单声道.
  int channel = 1;

  if (awaken_mode == READ_PCM_FILE_TO_ASR)
  {
    /***  选择已录好的pcm音频文件进行唤醒测试  ***/
    // 启动检测是否已被唤醒的线程
    std::thread check_0(isWaken_file);
    std::cout << "请输入用于测试的pcm文件,若输入为d，则用默认的文件进行唤醒测试:";
    std::cin >> pcm_file;

    if (pcm_file == "d")
    {
      pcm_file = base_path + "/defaultconfig/audio/awaken.pcm";
      // 科大讯飞离线唤醒登录以及参数设置
      awaken_demo.loginAndSetParams(appid,base_path, pcm_file, ONE_CHANNEL);
    }
    else
    {
      // 科大讯飞离线唤醒登录以及参数设置
      awaken_demo.loginAndSetParams(appid,base_path, pcm_file, ONE_CHANNEL);
    }
    // 离线唤醒模块的初始化
    awaken_demo.awakenInit();
    // 获取pcm_file的数据
    awaken_demo.getPcmFileData();
    // 将pcm数据循环进行唤醒检测
    awaken_demo.dataLoopAwaken();
    check_0.join();
    // 一次离线唤醒结束后释放资源
    awaken_demo.uninitAsr();
  }
  else if (awaken_mode == RECORDE_TO_ASR)
  {
    /*** 通过实时录音进行测试  ***/
    // 启动检测是否已被唤醒的线程
    std::thread check_1(isWaken_mic);
    // 科大讯飞离线唤醒登录以及参数设置
    awaken_demo.loginAndSetParams(appid,base_path, pcm_file, channel);
    // 离线唤醒模块的初始化
    awaken_demo.awakenInit();
    // record_time 录音时长(s),可修改
    float record_time = 120;
    // enable_record_save是否将录音保存到pcm文件
    bool enable_record_save = true;
    // 录音并进行语音唤醒
    awaken_demo.recordThroughMIC(record_time, enable_record_save);
    check_1.join();
    // 一次离线唤醒结束后释放资源
    awaken_demo.uninitAsr();
  }
  else
  {
    std::cout << "无效的awaken_mode模式的选择!!!" << std::endl;
    exit(1);
  }
}
