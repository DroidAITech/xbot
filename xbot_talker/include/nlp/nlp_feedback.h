/**
 * @file    nlp_feedback.h
 * @brief   语音识别结果处理与机器人交互模块.
 * TODO: 还需要添加版权、版本等信息
 * @author  Xiaoyun Wang.
 */
#ifndef RESULT_FEEDVACK_HPP
#define RESULT_FEEDVACK_HPP
#include <curl/curl.h>
#include <fstream>
#include <iostream>
#include <vector>

/** 要执行的机器人动作,行为字典 */
enum RobotAciton
{
  NO_ACTION = 0,
  CLOSE_CHAT = 1,
  START_CHAT = 2,
  TAKE_A_STEP_FORWARD = 101,
  TAKE_A_STEP_BACKWARD =102,
  TURN_LEFT = 103,
  TURN_RIGHT = 104,
  LOOK_UP = 105,
  LOOK_DOWN = 106,
  LOOK_LEFT =107,
  LOOK_RIGHT = 108,
  LOOK_FORWARD = 109,

  RESET_ARM = 201,
  RAISE_LEFT_HAND = 202,
  PUT_DOWM_LEFT_HAND = 203,
  RAISE_RIGHT_HAND = 204,
  PUT_DOWM_RIGHT_HAND = 205,
  LEFT_HAND_GRIP = 206,
  LEFT_HAND_OPEN =207,
  RIGHT_HAND_GRIP = 208,
  RIGHT_HAND_OPEN =209,
  RAISE_ARM = 210,
  PUT_DOWN_ARM = 211,
  OPEN_GRIPPER = 212,
  CLOSE_GRIPPER = 213,


  NAVI_START = 301,
  NAVI_TO_POSE = 302,
  NAVI_CANCEL = 303,
  NAVI_CONTINUE = 304,
  BACK_TO_ORIGINAL_LOCATION = 305,
  NAVI_TO_FIRST_POSE = 311,
  NAVI_TO_SECOND_POSE = 312,
  NAVI_TO_THIRD_POSE = 313,
  NAVI_TO_FOURTH_POSE = 314,
  NAVI_TO_FIFTH_POSE = 315,

};

/** 动作模式字典 */
enum ActionMode
{
  PLAY_ONLY = 0,
  ACTION_ONLY,
  PLAY_BEFORE_ACTION,
  PLAY_AND_ACTION_SYNC,
  PLAY_AFTER_ACTION,
};

/** 语音识别结果处理 */
class ResultFeedback
{
public:
  ResultFeedback() : answer_dictionary_(4)
  {
  }

  /**
   * @fn resultIsValid
   * @brief	判断语音识别结果是否为有效结果,若语音识别模块传来的结果！=none_result,则为有效.
   * @param recog_result			- [in] 语音识别结果.
   * @return bool             - 若语音识别模块传来的结果！=none_result,则则返回结果为true.
   */
  bool resultIsValid(const std::string recog_result);

  /**
   * @fn readAnswerTable
   * @brief	读取离线对话库：defaultconfig/answer_dic.csv文件里的内容，存入answer_table_.
   * @param answer_config_file			- [in] 离线对话配置文件.
   * @return std::vector<std::vector<std::string>>  answer_table_    - 返回存储对话信息的vector.
   */
  std::vector<std::vector<std::string>> readAnswerTable(std::string answer_config_file);

  /**
   * @fn keywordDetection
   * @brief	根据交互模式answer_table_判断识别出的关键词是否有对应的响应策略.
   *        若该关键词有对应的响应回复,存入响应向量answer_dictionary_(4).
   * @return std::vector<std::string>    - 若存在响应，返回一整行响应策略，若不存在，则返回向量的空.
   */
  std::vector<std::string> keywordDetection();

  /**
   * @fn keywordResponse
   * @brief	根据响应vector内的值调用不同的响应接口，如发布控制指令，播放音频文件等.
   * @param log_path			- [in] 语音合成的音频文件的缓存路径.
   */
  void keywordResponse(const std::string log_path);

  /**
   * @fn answerContent
   * @brief	返回一个反馈内容（包含语音和动作的反馈）.
   * @return std::vector<std::string> answer_dictionary_     - 返回answer_dictionary_
   */
  std::vector<std::string> answerContent();

  /**
   * @fn actionDetect
   * @brief	监测是否需要机器人控制以及控制类型..
   * @return int     - 返回RobotAciton.
   */
  int actionDetect();

  /**
   * @fn isKeywordRepeated
   * @brief	判断新加的关键词是否已被定义过.
   * @return bool     - 若已被定义过，则返回true.
   */
  bool isKeywordRepeated(std::string key_word);

  /**
   * @fn uninitNLP
   * @brief	一次处理结束后释放资源.
   */
  void uninitNLP();

private:
  std::vector<std::string> answer_dictionary_;          // 存储一个关键词及其对应的反馈内容
  std::vector<std::vector<std::string>> answer_table_;  // 存储整个csv表格的内容
  int detect_count_ = 0;                                // 存储识别到的关键词存在的回复的个数
  std::string recog_final_result_;                      // 从json串中解析出的语音识别结果
  std::string base_path_;
  std::string tts_path_;
  void robotPlay();
  int robotAction();
};

/** 图灵对话机器人模块 */
class TuLingRobot
{
public:
  /**
   * @fn setAskJson
   * @brief	设置图灵机器人对话请求参数.
   * @param tuling_key			- [in] 用户注册图灵的key
   * @param ask_str			- [in] 提问的语句.
   */
  void setAskJson(const std::string tuling_key,const std::string ask_str);

  /**
   * @fn callTulingApi
   * @brief	请求图灵机器人接口，获取回答.
   * @return std::string answer_json_			- 返回图灵机器人的完整json回答.
   */
  std::string callTulingApi();

  /**
   * @fn textFromJson
   * @brief	从图灵机器人的完整json回答中解析出最终需要的text回答.
   * @return std::string answer_text_			- 返回需要的text回答.
   */
  std::string textFromJson();

  /**
   * @fn uninitTuling
   * @brief	一次回答后释放资源.
   */
  void uninitTuling();

private:
  std::string tuling_url_ = "http://openapi.tuling123.com/openapi/api/v2";
  std::string tuling_key_;
  std::string ask_json_;
  std::string answer_json_;
  std::string answer_text_;
  static size_t http_data_writer(void* data, size_t size, size_t nmemb, void* content);
};

#endif
