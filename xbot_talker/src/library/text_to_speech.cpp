#include "tts/text_to_speech.h"
#include <unistd.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include "alsa/asoundlib.h"
#include "common_config.h"
//静态map成员text_audio_map初始化
map<string, string> TextToSpeech::text_audio_map = TextToSpeech::createMap();

TextToSpeech::TextToSpeech()
{
  //暂时处理为本地绝对路径
  basePath = "/home/chenying/catkin_ws/src/xbot_talker";
  src_text = "你好，欢迎来到重德智能";
  audioFile = "greet_visitor.wav";
}

TextToSpeech::TextToSpeech(const char* src_text)
{
  src_text = src_text;
}

TextToSpeech::TextToSpeech(const string& basePath, const char* src_text, const string& audioFile)
{
  TextToSpeech::basePath = basePath;
  src_text = src_text;
  TextToSpeech::audioFile = audioFile;
  std::cout << "TextToSpeech::TextToSpeech(string basePath, const char* "
               "src_text, string audioFile) is called."
            << std::endl;
  std::cout << basePath << std::endl;
}

TextToSpeech::~TextToSpeech()
{
}

map<string, string> TextToSpeech::createMap()
{
  map<string, string> text_audio_tempmap;
  return text_audio_tempmap;
}

/**
 * 该函数生成音频文件存储路径
 * @param current_map 当前的文本与语音文件的映射
 * @return 指定的路径
 */
string TextToSpeech::getAudioFile(map<string, string> current_map)
{
  return basePath + "/" + std::to_string(current_map.size()) + ".wav";
}

/**
 * tts功能实现函数
 * @param src_text 要转换成播放语音的文字
 * @return 播放生成语音
 * - false表示转换失败
 */
bool TextToSpeech::audioConverter(const std::string base_path, const char* src_text)
{
  basePath = base_path;
  int ret = MSP_ERROR_FAIL;
  const char* sessionID = NULL;
  unsigned int audio_len = 0;
  wave_pcm_hdr wav_hdr = default_wav_hdr;
  int synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;


  //判断src_text是否在text_audio_map中，
  //若存在，则直接播放对应的音频文件
  //不存在，则调用讯飞tts接口生成相应的音频文件并保存到相应路径
  map<string, string>::iterator filter = text_audio_map.find(src_text);
  if (filter != text_audio_map.end())
  {
    //播放已存在的缓存文件
    play_wav( text_audio_map[src_text]);

    std::cout << "Playing the file cached done.   " << std::endl;
  }
  else
  {

    std::cout << "需要调用xunfei tts函数生成相应音频文件并保存" << std::endl;

    string tmp_file = getAudioFile(text_audio_map);
    std::cout << tmp_file << std::endl;
    const char* tts_begin_params = "engine_type = local,voice_name=xiaoyan, text_encoding = UTF8, "
                                   "tts_res_path = fo|res/tts/xiaoyan.jet;fo|res/tts/common.jet, "
                                   "sample_rate = 44100, speed = 50, volume = 50, pitch = 50, rdn = 2";
    if (NULL == src_text)
    {
      std::cout << "param is error!" << std::endl;
      return false;
    }
    std::ofstream outfile(tmp_file.c_str(), std::ios::out | std::ios::binary);
    if (!outfile.is_open())
    {
      std::cout << "open file path error " << tmp_file << std::endl;
      return false;
    }
    // 开始一次语音合成，分配语音合成资源。
    // 返回：函数调用成功返回字符串格式的sessionID，失败返回NULL
    // 参数：params[in]
    // 传入的参数列表：
    // http://mscdoc.xfyun.cn/windows/api/iFlytekMSCReferenceManual/qtts_8h.html#a3fba4ad9599445073335851cc9479542
    // errorCode[out]	函数调用成功则其值为 MSP_SUCCESS，否则返回错误代码
    sessionID = QTTSSessionBegin(tts_begin_params, &ret);
    if (MSP_SUCCESS != ret)
    {
      std::cout << "QTTSSessionBegin failed, error code: " << ret << std::endl;
      outfile.close();
      return false;
    }
    // 写入要合成的文本。
    // 返回：函数调用成功则其值为 MSP_SUCCESS，否则返回错误代码
    // 参数：sessionID[in] 由QTTSSessionBegin返回的句柄。
    // textString[in]	字符串指针。指向待合成的文本字符串。
    // textLen[in] 合成文本长度,最大支持8192个字节（不含’\0’）。
    // params[in]	本次合成所用的参数，只对本次合成的文本有效。目前为空
    // 备注：本接口不支持连续被调用。调用本接口写入合成文本后，
    // 用户需要反复调用QTTSAudioGet 接口来获取音频。
    ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
    if (MSP_SUCCESS != ret)
    {
      std::cout << "QTTSTextPut failed, error code:" << ret << std::endl;
      QTTSSessionEnd(sessionID, "TextPutError");

      return false;
    }

    // 添加wav音频头，使用采样率为16000
    outfile.write(reinterpret_cast<char*>(&wav_hdr), sizeof(wav_hdr));
    // 获取合成音频
    // 返回：函数调用成功且有音频数据时返回非空指针。调用失败或无音频数据时，返回NULL。
    // 参数：sessionID[in]	由QTTSSessionBegin返回的句柄。
    // audioLen[out]	合成音频长度，单位字节。
    // synthStatus[out]	合成音频状态
    // http://mscdoc.xfyun.cn/windows/api/iFlytekMSCReferenceManual/qtts_8h.html#a4e4f6bed4b9e4ea553aa00ccf539c22a
    // errorCode[out]	函数调用成功则其值为MSP_SUCCESS，否则返回错误代码
    // 备注：用户需要反复获取音频，直到音频获取完毕或函数调用失败。在重复获取音频时，如果暂未获得音频数据，需要将当前线程sleep一段时间，以防频繁调用浪费CPU资源。
    while (1)
    {
      const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
      if (MSP_SUCCESS != ret)
        break;
      if (NULL != data)
      {
        outfile.write(static_cast<const char*>(const_cast<void*>(data)), audio_len);
        //计算data_size大小
        wav_hdr.data_size += audio_len;
      }
      if (MSP_TTS_FLAG_DATA_END == synth_status)
        break;
    }

    if (MSP_SUCCESS != ret)
    {
      std::cout << "QTTSAudioGet failed, error code: " << ret << std::endl;
      QTTSSessionEnd(sessionID, "AudioGetError");

      return false;
    }
    // 修正wav文件头数据的大小
    wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);

    // 将修正过的数据写回文件头部,音频文件为wav格式
    outfile.seekp(4, std::ios::beg);
    // 写入size_8的值
    outfile.write(reinterpret_cast<char*>(&wav_hdr.size_8), sizeof(wav_hdr.size_8));
    // 将文件指针偏移到存储data_size值的位置
    outfile.seekp(40, std::ios::beg);
    // 写入data_size的值
    outfile.write(reinterpret_cast<char*>(&wav_hdr.data_size), sizeof(wav_hdr.data_size));
    outfile.close();
    // 合成完毕，播放音频
    ret = QTTSSessionEnd(sessionID, "Normal");
    if (MSP_SUCCESS != ret)
    {
      std::cout << "QTTSSessionEnd failed, error code: " << ret << std::endl;
    }
    std::cout << "Synthesize completed." << std::endl;

    //保存相应文本和语音文件到text_audio_map
    text_audio_map.insert(std::make_pair(src_text, tmp_file));

    std::cout << "Start talking.......   " << tmp_file << std::endl;
    play_wav(tmp_file.c_str());
    std::cout << "Playing done.   " << std::endl;
  }

  return true;
}
int TextToSpeech::set_pcm_play(FILE* fp, wave_pcm_hdr* wav_header)
{
  int rc;
  int ret;
  int size;
  snd_pcm_t* handle;            // PCI设备句柄
  snd_pcm_hw_params_t* params;  //硬件信息和PCM流配置
  unsigned int val;
  int dir = 0;
  snd_pcm_uframes_t frames;
  char* buffer;
  int channels = wav_header->channels;
  int frequency = wav_header->samples_per_sec;
  int bit = wav_header->bits_per_sample;
  int datablock = wav_header->block_align;

  rc = snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
  if (rc < 0)
  {
    perror("\nopen PCM device failed:");
    exit(TTS_ERROR_OPEN_PCM_DEVICE_FAIL);
  }

  snd_pcm_hw_params_alloca(&params);  //分配params结构体
  if (rc < 0)
  {
    perror("\nsnd_pcm_hw_params_alloca:");
    exit(TTS_ERROR_SND_PCM_HW_PARAM_ALLOCA_FAIL);
  }

  rc = snd_pcm_hw_params_any(handle, params);  //初始化params
  if (rc < 0)
  {
    perror("\nsnd_pcm_hw_params_any:");
    exit(TTS_ERROR_SND_PCM_HW_PARAMS_ANY);
  }
  rc = snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);  //初始化访问权限
  if (rc < 0)
  {
    perror("\nsed_pcm_hw_set_access:");
    exit(TTS_ERROR_SND_PCM_HW_SET_ACCESS);
  }

  //采样位数
  switch (bit / 8)
  {
    case 1:
      snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_U8);
      break;
    case 2:
      snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
      break;
    case 3:
      snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S24_LE);
      break;
  }
  rc = snd_pcm_hw_params_set_channels(handle, params, channels);  //设置声道,1表示单声>道，2表示立体声
  if (rc < 0)
  {
    perror("\nsnd_pcm_hw_params_set_channels:");
    exit(TTS_ERROR_SET_CHANNEL_FAIL);
  }
  val = frequency;
  rc = snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir);  //设置>频率
  if (rc < 0)
  {
    perror("\nsnd_pcm_hw_params_set_rate_near:");
    exit(TTS_ERROR_RATE_SET_FAIL);
  }

  rc = snd_pcm_hw_params(handle, params);
  if (rc < 0)
  {
    perror("\nsnd_pcm_hw_params: ");
    exit(TTS_ERROR_SET_HW_PARAM_FAIL);
  }

  rc = snd_pcm_hw_params_get_period_size(params, &frames, &dir); /*获取周期长度*/
  if (rc < 0)
  {
    perror("\nsnd_pcm_hw_params_get_period_size:");
    exit(TTS_ERROR_GET_PERIOD_SIZE_FAIL);
  }

  size = frames * datablock; /*4 代表数据块长度*/

  buffer = (char*)malloc(size);
  if (buffer == NULL)
  {
    std::cout << "\n Malloc failed in tts module::buffer !!!\n";
    exit(TTS_ERROR_MALLOC_FAIL);
  }
  fseek(fp, 58, SEEK_SET);  //定位歌曲到数据区,从起始位置偏移量为58.

  while (true)
  {
    memset(buffer, 0, sizeof(buffer));  //初始化新申请的内存

    ret = fread(buffer, 1, size, fp);
    if (ret == 0)
    {
      break;
    }
    else if (ret != size)
    {
      break;
    }
    // 写音频数据到PCM设备
    snd_pcm_writei(handle, buffer, frames);

    if (ret == -EPIPE)
    {
      // EPIPE means underrun
      fprintf(stderr, "underrun occurred\n");
      //完成硬件参数设置，使设备准备好
      snd_pcm_prepare(handle);
    }
    else if (ret < 0)
    {
      fprintf(stderr, "error from writei: %s\n", snd_strerror(ret));
    }
    usleep(2 * 1000);
  }

  snd_pcm_drain(handle);
  snd_pcm_close(handle);
  free(buffer);
  return 0;
}

int TextToSpeech::play_wav(const string& file_path)
{
  FILE* fp;
  wave_pcm_hdr wav_header;
  fp = fopen(file_path.c_str(), "rb");
  if (fp == NULL)
  {
    return TTS_ERROR_FILE_OPEN_FAIL;
  }
  int nread = fread(&wav_header, 1, sizeof(wav_header), fp);
  set_pcm_play(fp, &wav_header);
  free(fp);
}
