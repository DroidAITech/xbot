#include "awaken/awaken_offline.h"
#include <thread>
#include "common_config.h"
#include "linuxrec.h"
#include "asr/xunfei/msp_cmn.h"
#include "asr/xunfei/msp_errors.h"
#include "asr/xunfei/msp_types.h"
#include "asr/xunfei/qivw.h"
#include <cstring>

char* AwakenOffline::awaken_result_ = NULL;
bool AwakenOffline::is_awaken = false;
void AwakenOffline::checkIsAwaken()
{
  while (true)
  {
    if (is_awaken)
    {
      //播放“嘟”声提示音.
      std::string dialogue_prompt_wav = "play " + base_path_ + "/defaultconfig/audio/call.wav";
      system(dialogue_prompt_wav.c_str());

      is_awaken = false;
      break;
    }
    else
    {
      continue;
    }
  }
}

// 科大讯飞登录以及语音唤醒相关参数的设置
void AwakenOffline::loginAndSetParams(const std::string appid, const std::string base_path, const std::string pcm_file, const int channel)
{
  // 公司只买了语音合成以及离线关键词识别，appid对应的语音唤醒设备已经超出免费版范围（3台设备），
  // 于是自己注册了科大讯飞申请了新的appid，于是对应的libmsc.so和msc都需要重新的资源。
  // FIXME:购买语音唤醒功能后需替换appid和sdk资源
  std::string login_parameters = "appid = " + appid + ", work_dir = " + base_path + "/defaultconfig";
  int error_code = MSPLogin(NULL, NULL, login_parameters.c_str());
  if (MSP_SUCCESS != error_code)
  {
    std::cout << "MSPLogin failed ! errorcode:" << error_code << std::endl;
    exit(AWAKEN_ERROR_XFEI_API_FAIL);
  }
  awaken_params_ = "ivw_threshold=0:1450,sst=wakeup,ivw_res_path "
                   "=fo|res/ivw/wakeupresource.jet";
  base_path_ = base_path;
  pcm_file_ = pcm_file;
  channel_ = channel;
}

// 一次对话结束后释放资源
void AwakenOffline::uninitAsr()
{
  // 结束本次语音唤醒
  int ret = QIVWSessionEnd(session_id_, "normal end");
  if (MSP_SUCCESS != ret)
  {
    std::cout << "QIVWSessionEnd failed !errorcode:" << ret << std::endl;
  }
}

// 从双声道的音频数据中分离出单声道数据
struct DataBuff AwakenOffline::getOneChannelData()
{
  struct DataBuff pcm_two_channel = pcm_data_;
  pcm_data_ = { NULL, 0 };
  pcm_data_.size = pcm_two_channel.size / 2;

  pcm_data_.data = new char[pcm_data_.size];
  for (int i = 0; i < pcm_data_.size / 2; i++)
  {
    memcpy((uint16_t*)pcm_data_.data + i, ((uint32_t*)(pcm_two_channel.data)) + i, 2);
  }
  return pcm_data_;
}

// 从用于语音唤醒的pcm音频文件中读取数据
void AwakenOffline::getPcmFileData()
{
  FileOperation pcm_file;
  pcm_data_ = pcm_file.readFileAsDatabuffer(pcm_file_);
}

void AwakenOffline::writeAudioData(const char* audio_data, unsigned int audio_len)
{
  int err_code;
  err_code = QIVWAudioWrite(session_id_, (const void*)audio_data, audio_len, audio_stat_);
  if (MSP_SUCCESS != err_code)
  {
    std::cout << "QIVWAudioWrite failed! error code:" << err_code << std::endl;
    exit(AWAKEN_ERROR_XFEI_API_FAIL);
  }
}
void sleep_ms(int ms)
{
  usleep(ms * 1000);
}

// 调用回调函数通知唤醒成功息同时给出相应唤醒数据。
// 如果出错，msc 调用回调函数给出错误信息。
int AwakenOffline::cb_ivw_msg_proc(const char* sessionID, int msg, int param1, int param2, const void* info,
                                   void* userData)
{
  //唤醒出错消息
  if (MSP_IVW_MSG_ERROR == msg)
  {
    std::cout << "\nMSP_IVW_MSG_ERROR errCode = " << param1 << std::endl;
    exit(AWAKEN_ERROR_XFEI_API_FAIL);
  }
  //唤醒成功消息
  else if (MSP_IVW_MSG_WAKEUP == msg)
  {
    std::cout << "\nMSP_IVW_MSG_WAKEUP result = " << (const char*)info << std::endl;
    awaken_result_ = (char*)info;
    is_awaken = true;
    awaken_result_ = NULL;
  }
  return 0;
}

void AwakenOffline::awakenInit()
{
  int err_code = MSP_SUCCESS;
  session_id_ = QIVWSessionBegin(NULL, awaken_params_, &err_code);
  if (err_code != MSP_SUCCESS)
  {
    std::cout << "QIVWSessionBegin failed! error code:" << err_code << std::endl;
    exit(AWAKEN_ERROR_XFEI_API_FAIL);
  }

  err_code = QIVWRegisterNotify(session_id_, cb_ivw_msg_proc, NULL);
  if (err_code != MSP_SUCCESS)
  {
    std::cout << "QIVWRegisterNotify failed! error code:" << err_code << std::endl;
    exit(AWAKEN_ERROR_XFEI_API_FAIL);
  }
}

// 把一段长pcm音频数据循环写入qivw
void AwakenOffline::dataLoopAwaken()
{
  long real_read = 0;
  long audio_count = 0;
  int count = 0;
  audio_stat_ = MSP_AUDIO_SAMPLE_CONTINUE;
  // 根据音频数据是单声道还是双声道选择接口
  if (channel_ == 2)
  {
    getOneChannelData();
  }
  while (1)
  {
    long len = 10 * FRAME_LEN;  // 16k音频，10帧 （时长200ms）
    audio_stat_ = MSP_AUDIO_SAMPLE_CONTINUE;
    if (pcm_data_.size <= len)
    {
      len = pcm_data_.size;
      audio_stat_ = MSP_AUDIO_SAMPLE_LAST;  //最后一块
    }
    if (0 == audio_count)
    {
      audio_stat_ = MSP_AUDIO_SAMPLE_FIRST;
    }
    writeAudioData(&pcm_data_.data[audio_count], len);
    if (MSP_AUDIO_SAMPLE_LAST == audio_stat_)
    {
      break;
    }
    audio_count += len;
    pcm_data_.size -= len;
    sleep_ms(200);  //模拟人说话时间间隙，10帧的音频时长为200ms
  }
  free(pcm_data_.data);
}

void AwakenOffline::saveRecordDataToFile()
{
  std::string save_file_name = file_operation.setFileName("-awaken.pcm");
  std::string save_file = base_path_ + "/cache/pcm" + save_file_name;
  std::ofstream pcm_file(save_file, std::ofstream::binary);
  pcm_file.write(pcm_data_.data, pcm_data_.size);
  pcm_file.close();
}

void AwakenOffline::recordThroughMIC(const float record_time, bool enable_audio_save)
{
  int success_code = 0;
  int errorcode;
  pcm_data_.data = NULL;
  pcm_data_.size = 0;
  std::cout << "-----------Start Voice Wake up Thread --------" << std::endl;
  // 采用默认设备获取音频
  record_dev_id device_id = getDefaultInputDevice();
  int errcode = 0;
  // 使用WAVEFORMATEX结构指定pcm数据格式。
  WAVEFORMATEX wavfmt = { WAVE_FORMAT_PCM, 1, 16000, 32000, 2, 16, sizeof(WAVEFORMATEX) };
  if (getInputDeviceNum() == 0)
  {
    std::cout << "\nNo active record device find! ";
  }
  else
  {
    std::cout << "The total number of active input devices is : " << getInputDeviceNum() << std::endl;
  }
  // 设置myrec用于存储录音信息.
  awaken_record_ = (struct recorder*)malloc(sizeof(struct recorder));

  memset(awaken_record_, 0, sizeof(struct recorder));

  awaken_record_->state = RECORD_STATE_CREATED;

  awaken_record_->pcm_file_path = base_path_ + "/cache/pcm";
  audio_stat_ = MSP_AUDIO_SAMPLE_CONTINUE;

  record_alsa.initRecord(awaken_record_, device_id, &wavfmt);
  record_loops_ = record_alsa.setRecordDuration(record_time);

  int buf_count = 0;  //分段录音计数
  struct DataBuff record_pcm;
  while (record_loops_ > 0)
  {
    record_pcm = record_alsa.startRecord();
    writeAudioData(record_pcm.data, record_pcm.size);
    record_loops_--;
    pcm_data_.data = (char*)realloc(pcm_data_.data, record_pcm.size * (buf_count + 1));

    if (pcm_data_.data == NULL)
    {
      std::cout << "ERROR:buf_new realloc error!" << std::endl;
      exit(AWAKEN_ERROR_XFEI_API_FAIL);
    }
    std::memcpy(&pcm_data_.data[record_pcm.size * buf_count], record_pcm.data, record_pcm.size);
    buf_count += 1;
    pcm_data_.size = record_pcm.size * buf_count;
  }
}

void AwakenOffline::stopRecordThroughMIC()
{
  record_loops_ = 0;
  record_alsa.closeRecord();
}
