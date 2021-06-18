#include "asr/baidu/asr_online.h"
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include "rapidjson/document.h"
#include "rapidjson/rapidjson.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
void BaiduAsrOnline::praseToken()
{
  rapidjson::Document doc;
  doc.Parse(url_response_);
  if (doc.HasParseError())
  {
    rapidjson::ParseErrorCode code = doc.GetParseError();
    std::cout << "JSON解析错误" << code << std::endl;
    exit(ASR_ERROR_JSON_PARSE_FAIL);
  }
  else
  {
    rapidjson::Value& vConfidence = doc["access_token"];
    token_ = vConfidence.GetString();
    std::cout << "token from url_response json is :" << token_ << std::endl;
  }
}

void BaiduAsrOnline::setAsrParams(std::string baidu_api_key, std::string baidu_srcret_key,const std::string base_path, const std::string pcm_file, const int channel)
{
  base_path_ = base_path;
  pcm_file_ = pcm_file;
  channel_ = channel;
  asr_config.api_key = baidu_api_key;
  asr_config.secret_key = baidu_srcret_key;
}
void BaiduAsrOnline::initAndConfigAsr()
{
  // CURL初始化
  curl_global_init(CURL_GLOBAL_ALL);
  // 配置asr相关参数
  // 填写网页上申请的appkey 如 g_api_key="g8eBUMSokVB1BHGmgxxxxxx"
  //asr_config.api_key = "kVcnfD9iW2XVZSMaLMrtLYIz";
  // 填写网页上申请的APP SECRET 如
  // $secretKey="94dc99566550d87f8fa8ece112xxxxx"
  //asr_config.secret_key = "O9o1O213UgG5LFn0bDGNtoRN3VWl2du6";
  // 文件后缀仅支持 pcm/wav/amr 格式，极速版额外支持m4a 格式
  asr_config.format = "pcm";
  asr_config.url = "http://vop.baidu.com/server_api";  // 可改为https
  // 1537 表示识别普通话，使用输入法模型。1536表示识别普通话，使用搜索模型
  asr_config.dev_pid = 1537;
  // 有此scope表示有asr能力，没有请在网页里勾选，非常旧的应用可能没有
  asr_config.scope = "audio_voice_assistant_get";
  // 采样率固定值
  asr_config.rate = 16000;
  asr_config.cuid = "1234567C";
}
void BaiduAsrOnline::speechGetToken()
{
  std::string url = API_TOKEN_URL + "?grant_type=client_credentials&client_id=" + asr_config.api_key +
                    "&client_secret=" + asr_config.secret_key;
  std::cout << "URL is : " << url << std::endl;
  CURL* curl = curl_easy_init();

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());  // 注意返回值判读
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60);  // 60s超时
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writefunc);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &url_response_);
  CURLcode res_curl = curl_easy_perform(curl);
  int res = ASR_RETURN_OK;
  if (res_curl != CURLE_OK)
  {
    std::cout << "perform curl error: " << res << "," << curl_easy_strerror(res_curl) << std::endl;
    exit(ASR_ERROR_TOKEN_CURL);
  }
  else
  {
    // 解析token，结果保存在token里
    praseToken();
  }
  free(url_response_);
  curl_easy_cleanup(curl);
}

void BaiduAsrOnline::getPcmFileData()
{
  pcm_data_ = pcm_file_operation.readFileAsDatabuffer(pcm_file_);
}

char* BaiduAsrOnline::runAsrAndRecog()
{
  // 根据音频数据是单声道还是双声道选择接口
  if (channel_ == 2)
  {
    getOneChannelData();
  }
  CURL* curl = curl_easy_init();                                                                       // 需要释放
  char* cuid = curl_easy_escape(curl, asr_config.cuid.c_str(), std::strlen(asr_config.cuid.c_str()));  // 需要释放

  std::string url =
      asr_config.url + "?cuid=" + cuid + "&token=" + token_ + "&dev_pid=" + std::to_string(asr_config.dev_pid);
  std::cout << "runAsrAndRecog:: url = :" << url << std::endl;

  free(cuid);

  struct curl_slist* headerlist = NULL;
  std::string header;
  header = "Content-Type: audio/" + asr_config.format + "; rate=" + std::to_string(asr_config.rate);
  headerlist = curl_slist_append(headerlist, header.c_str());  // 需要释放
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_POST, 1);
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5);              // 连接5s超时
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60);                    // 整体请求60s超时
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);         // 添加http header Content-Type
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, pcm_data_.data);     // 音频数据
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, pcm_data_.size);  // 音频数据长度
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writefunc);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &rec_result_);  // 需要释放

  CURLcode res_curl = curl_easy_perform(curl);
  std::cout << "Baidu online speech recognition result is :" << rec_result_ << std::endl;
  curl_slist_free_all(headerlist);
  pcm_data_.data = NULL;
  pcm_data_.size = 0;
  curl_easy_cleanup(curl);
  return rec_result_;
}

// libcurl 返回回调
size_t BaiduAsrOnline::writefunc(void* ptr, size_t size, size_t nmemb, char** result)
{
  size_t result_len = size * nmemb;
  int is_new = (*result == NULL);
  if (is_new)
  {
    *result = (char*)malloc(result_len + 1);
    if (*result == NULL)
    {
      std::cout << "realloc failure!\n";
      return 1;
    }
    memcpy(*result, ptr, result_len);
    (*result)[result_len] = '\0';
  }
  else
  {
    size_t old_size = strlen(*result);
    *result = (char*)realloc(*result, result_len + old_size);
    if (*result == NULL)
    {
      std::cout << "realloc failure!\n";
      return 1;
    }
    memcpy(*result + old_size, ptr, result_len);
    (*result)[result_len + old_size] = '\0';
  }
  return result_len;
}

void BaiduAsrOnline::getPCMData(struct DataBuff pcm_buff)
{
  pcm_data_ = pcm_buff;
}

// 从双声道的音频数据中分离出单声道数据
struct DataBuff BaiduAsrOnline::getOneChannelData()
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

void BaiduAsrOnline::saveRecordDataToFile()
{
  std::string save_file_name = pcm_file_operation.setFileName("-asr.pcm");
  std::string save_file = base_path_ + "/cache/pcm" + save_file_name;
  std::ofstream pcm_file(save_file, std::ofstream::binary);
  pcm_file.write(pcm_data_.data, pcm_data_.size);
  pcm_file.close();
}

struct DataBuff BaiduAsrOnline::recordThroughMIC(const float record_time, bool enable_audio_save)
{
  int success_code = 0;
  int errorcode;
  std::cout << "-----------Start ASR Recording Thread --------" << std::endl;
  // 采用默认设备获取音频
  record_dev_id device_id = getDefaultInputDevice();
  int errcode = 0;
  // 使用WAVEFORMATEX结构指定pcm数据格式。
  WAVEFORMATEX wavfmt = { WAVE_FORMAT_PCM, 1, 16000, 32000, 2, 16, sizeof(WAVEFORMATEX) };
  if (getInputDeviceNum() == 0)
  {
    std::cout << "\nNo active record device find! " << std::endl;
  }
  else
  {
    std::cout << "The total number of active input devices is : " << getInputDeviceNum() << std::endl;
  }
  // 设置myrec用于存储录音信息.
  asr_record_ = (struct recorder*)malloc(sizeof(struct recorder));

  memset(asr_record_, 0, sizeof(struct recorder));

  asr_record_->state = RECORD_STATE_CREATED;

  asr_record_->pcm_file_path = base_path_ + "/cache/pcm";
  record_alsa.initRecord(asr_record_, device_id, &wavfmt);
  record_loops_ = record_alsa.setRecordDuration(record_time);

  int buf_count = 0;  //分段录音计数
  struct DataBuff record_pcm;
  while (record_loops_ > 0)
  {
    record_pcm = record_alsa.startRecord();
    record_loops_--;
    pcm_data_.data = (char*)realloc(pcm_data_.data, record_pcm.size * (buf_count + 1));

    if (pcm_data_.data == NULL)
    {
      std::cout << "ERROR:buf_new realloc error!" << std::endl;
      exit(ASR_ERROR_REALLOC_FAIL);
    }
    std::memcpy(&pcm_data_.data[record_pcm.size * buf_count], record_pcm.data, record_pcm.size);
    buf_count += 1;
  }
  pcm_data_.size = record_pcm.size * buf_count;
  // 根据选项选择是否保存录下的音频数据到文件。
  if (enable_audio_save)
  {
    std::thread save_pcm_to_file(&BaiduAsrOnline::saveRecordDataToFile, this);
    save_pcm_to_file.detach();
  }
  return pcm_data_;
}
std::string BaiduAsrOnline::resultFromJson()
{
  rapidjson::Document doc;
  doc.Parse(rec_result_);
  if (doc.HasParseError())
  {
    rapidjson::ParseErrorCode code = doc.GetParseError();
    std::cout << "JSON解析错误" << code << std::endl;
    exit(ASR_ERROR_JSON_PARSE_FAIL);
  }
  if (doc.HasMember("result"))
  {
    rapidjson::Value& result = doc["result"];

    online_final_result = result[0].GetString();
    std::cout << "online_final_result is :" << online_final_result << std::endl;
  }
  return online_final_result;
}

void BaiduAsrOnline::stopRecordThroughMIC()
{
  record_loops_ = 0;
  record_alsa.closeRecord();
}

// 一次对话结束后释放资源
void BaiduAsrOnline::uninitAsr()
{
  if (rec_result_)
  {
    free(rec_result_);
    rec_result_ = NULL;
    online_final_result = " ";
  }
}
