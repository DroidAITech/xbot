#include "nlp/nlp_feedback.h"
#include <fstream>
#include <iostream>
#include <vector>
#include "common_config.h"
#include "file_operation.h"
#include "rapidjson/document.h"
#include "rapidjson/rapidjson.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "tts/text_to_speech.h"
std::istream& operator>>(std::istream& str, CSVOperation& data)
{
  data.readNextRow(str);
  return str;
}
// 判断sub到的识别结果是否有效
bool ResultFeedback::resultIsValid(const std::string recog_result)
{
  if (recog_result == "none_result")
  {
    return false;
  }
  else
  {
    recog_final_result_ = recog_result;
    return true;
  }
}

bool ResultFeedback::isKeywordRepeated(std::string key_word)
{
  for (int i = 0; i < (answer_table_.size() - 1); i++)
  {
    if (key_word == answer_table_[i][0])
    {
      return true;
    }
  }

  return false;
}

void ResultFeedback::uninitNLP()
{
  recog_final_result_ = "";
  answer_dictionary_[0] = "";
}

// 根据交互模式配置文件检查识别出的关键词是否有对应的响应策略.
std::vector<std::string> ResultFeedback::keywordDetection()
{
  for (int i = 0; i < (answer_table_.size() - 1); i++)
  {
    if (recog_final_result_ == answer_table_[i][0])
    {
      for (int j = 0; j < 4; j++)
      {
        answer_dictionary_[j] = answer_table_[i][j];
      }
    }
  }

  if (answer_dictionary_[0] == "")
  {
    std::cout << "Can not find the answer corresponding to the keyword. Please check "
                 "the configuration file "
              << std::endl;
    //return answer_dictionary_;
  }
  return answer_dictionary_;
}

// 读取csv数据并存入answer_table_.
std::vector<std::vector<std::string>> ResultFeedback::readAnswerTable(std::string answer_config_file)
{
  std::ifstream file(answer_config_file);
  CSVOperation config_csv;
  answer_table_ = config_csv.readAllCSV(file);
  return answer_table_;
}

// 返回一个反馈内容（包含语音和动作的反馈）
std::vector<std::string> ResultFeedback::answerContent()
{
  return answer_dictionary_;
}

// TODO and FIXME:
// 在xbot_sdk完成后，控制机器人可以直接调用xbot里的接口函数而非通过ROStopic控制
// 因为xbot_talker后续也需要构建sdk，所以nlp代码不应该包含ros相关。
// 为实现功能，将机器人控制的话题发布统一放在xbot_talker_ros里.
int ResultFeedback::robotAction()
{
  return std::stoi(answer_dictionary_[2]);
}

// 监测是否需要机器人控制以及控制类型.
// FIXME:sdk完成后，此接口无需存在
int ResultFeedback::actionDetect()
{
  return robotAction();
}

// 此模式下应直接调用tts模块将answer_dictionary[1]里的answer文字转换成语音并播放
void ResultFeedback::robotPlay()
{
  TextToSpeech text_to_speech;
  bool ret;
  ret = text_to_speech.audioConverter(tts_path_, answer_dictionary_[1].c_str());
}

// 根据响应vector内的值调用不同的响应接口，如发布控制指令，播放音频文件等。
void ResultFeedback::keywordResponse(const std::string log_path)
{
  // csv表格的第三列代表action动作，第四列代表action_mode。
  tts_path_ = log_path;
  int action_mode = std::stoi(answer_dictionary_[3]);
  std::cout << "Action mode is:  " << action_mode << std::endl;
  std::cout << "Action mode is:  " << answer_dictionary_[3] << std::endl;
  switch (action_mode)
  {
    case PLAY_BEFORE_ACTION:
      robotPlay();
      robotAction();
      break;

    case PLAY_AFTER_ACTION:
      // TODO;
      break;

    case PLAY_AND_ACTION_SYNC:
      // TODO;
      break;

    case PLAY_ONLY:
      robotPlay();
      break;
    case ACTION_ONLY:
      robotAction();
      break;
    default:
      std::cout << "Error occurred in action_mode: " << action_mode << std::endl;
      break;
  }
}

// 图灵请求参数格式为json，UTF-8编码.
void TuLingRobot::setAskJson(const std::string tuling_key,const std::string ask_str)
{
  tuling_key_ = tuling_key;
  ask_json_ = "{\"reqType\": 0, \"perception\": {\"inputText\" : {\"text\": "
              "\"" +
              ask_str + "\"},"
                        "\"selfInfo\": {\"location\": {\"city\": \"北京\", \"province\": "
                        "\"\",\"street\": \"\"}}}, \"userInfo\": "
                        "{\"apiKey\":\"" +
              tuling_key_ + "\", "
                            "\"userId\": \"0\"}}";
}

size_t TuLingRobot::http_data_writer(void* data, size_t size, size_t nmemb, void* content)
{
  long totalSize = size * nmemb;
  std::string* symbolBuffer = (std::string*)content;
  if (symbolBuffer)
  {
    symbolBuffer->append((char*)data, ((char*)data) + totalSize);
  }
  return totalSize;
}

// HTTP POST请求图灵接口
std::string TuLingRobot::callTulingApi()
{
  int ret = 0;
  CURL* pCurl = NULL;
  CURLcode res;
  curl_global_init(CURL_GLOBAL_ALL);
  pCurl = curl_easy_init();
  if (NULL != pCurl)
  {
    curl_easy_setopt(pCurl, CURLOPT_TIMEOUT, 2);
    curl_easy_setopt(pCurl, CURLOPT_URL, tuling_url_.c_str());
    curl_slist* plist = curl_slist_append(NULL, "Content-Type:application/json;charset=UTF-8");
    curl_easy_setopt(pCurl, CURLOPT_HTTPHEADER, plist);
    std::cout << "Ask Json String is:" << ask_json_ << std::endl;

    curl_easy_setopt(pCurl, CURLOPT_POSTFIELDS, ask_json_.c_str());

    curl_easy_setopt(pCurl, CURLOPT_WRITEFUNCTION, http_data_writer);
    curl_easy_setopt(pCurl, CURLOPT_WRITEDATA, (void*)&answer_json_);
    res = curl_easy_perform(pCurl);
    long responseCode = 0;
    curl_easy_getinfo(pCurl, CURLINFO_RESPONSE_CODE, &responseCode);
    if (res != CURLE_OK)
    {
      std::cout << "curl_easy_perform() failed:" << curl_easy_strerror(res) << std::endl;
      exit(NLP_ERROR_CURL);
    }
    curl_easy_cleanup(pCurl);
  }
  curl_global_cleanup();
  std::cout << "Tuling robot answer:" << answer_json_ << std::endl;

  return answer_json_;
}

std::string TuLingRobot::textFromJson()
{
  std::string result = answer_json_;
  rapidjson::Document document;
  document.Parse(answer_json_.c_str());

  if (document.HasParseError())
  {
    rapidjson::ParseErrorCode code = document.GetParseError();
    std::cout << "JSON解析错误" << code << std::endl;
    exit(NLP_ERROR_JSON_PARSE_FAIL);
  }

  if (document.HasMember("results"))
  {
    const rapidjson::Value& childValue = document["results"];

    for (rapidjson::SizeType i = 0; i < childValue.Size(); ++i)
    {
      const rapidjson::Value& resultUnit = childValue[i];
      if (resultUnit.HasMember("values"))
      {
        const rapidjson::Value& contentWord = resultUnit["values"];

        const rapidjson::Value& text = contentWord["text"];
        answer_text_ = text.GetString();
        std::cout << answer_text_ << std::endl;
      }
    }
  }
  return answer_text_;
}

void TuLingRobot::uninitTuling()
{
  ask_json_ = "";
  answer_json_ = "";
  answer_text_ = "";
}
