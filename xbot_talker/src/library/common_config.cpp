#include "common_config.h"
#include <iostream>
#include "asr/xunfei/BuildGrammar.h"
#include "asr/xunfei/msp_cmn.h"
#include "asr/xunfei/msp_errors.h"
// MSP_login讯飞登录模块
void CommonConfig::loginToXunfei(const std::string base_path)
{
  // 讯飞 sdk 中注册的 appid 需要与下载的文件对应，否则报错
  std::string login_parameters = "appid = 5ade9569, work_dir = " + base_path + "/defaultconfig";
  int error_code = MSPLogin(NULL, NULL, login_parameters.c_str());
  if (MSP_SUCCESS != error_code)
  {
    std::cout << "MSPLogin failed :" << handleErrorcode(error_code) << "errorcode:" << error_code << std::endl;
    exit(ERROR_XUNFEI_LOGIN_FAIL);
  }
}

// 读取 config，存入 UserData 中
static void readConfig(const string file_path, UserData* info)
{
  std::ifstream infile(file_path);
  if (infile.is_open())
  {
    infile >> info->build_fini;
    infile >> info->update_fini;
    infile >> info->errcode;
    infile >> info->grammar_id;
    infile.close();
  }
  else
  {
    std::cout << "Fail to open expected Json file" << file_path << std::endl;
    exit(ERROR_FILE_OPEN_FAIL);
  }
}

// 设置离线语法参数
std::string CommonConfig::configGramParas(const std::string base_path, const std::string grammar_file)
{
  UserData asr_data;
  //讯飞语音sdk参数配置
  const std::string ASR_RES_PATH = "fo|res/asr/common.jet";
  const std::string GRAMMAR_FILE = grammar_file;  // 到/config的绝对路径
  const std::string GRAMMAR_BUILD_PATH = GRAMMAR_FILE;

  // 读入语法构建数据，存入 Userdata 中
  std::string filepath = GRAMMAR_BUILD_PATH + "/info.txt";
  readConfig(filepath, &asr_data);
  // 离线语法识别参数设置
  return std::string("engine_type = local, "
                     "asr_denoise=1,vad_bos=10000,vad_eos=10000,asr_res_path =" +
                     ASR_RES_PATH + ", sample_rate = " + to_string(SAMPLE_RATE_16K) + ", grm_build_path = " +
                     GRAMMAR_BUILD_PATH + ", local_grammar = " + asr_data.grammar_id +
                     ", result_type = json, result_encoding = UTF-8 ");
};

// 科大讯飞API返回值的处理:
// 错误码参考msp_errors.h。将几个用到的API中常见的错误码转换成了文字描述。
std::string CommonConfig::handleErrorcode(const int errorcode)
{
  while ((errorcode > MSP_SUCCESS) && (errorcode <= SPEECH_ERROR_LFASR_BASE))
  {
    switch (errorcode)
    {
      case MSP_ERROR_INVALID_PARA:
        return "Invalid parameters!";

      case MSP_ERROR_INVALID_PARA_VALUE:
        return "Invalid parameter value！";

      case MSP_ERROR_DB_INVALID_USER:
        return "Invalid username！";

      case MSP_ERROR_DB_INVALID_PWD:
        return "Invalid password！";

      case MSP_ERROR_DB_INVALID_APPID:
        return "Invalid app ID！";

      case MSP_ERROR_NOT_INIT:
        return "Not initialized！";

      case MSP_ERROR_CREATE_HANDLE:
        return "Fail to create session instance！";

      case MSP_ERROR_INVALID_HANDLE:
        return "Invalid session ID!";

      case MSP_ERROR_NO_DATA:
        return "NO data!";

      case MSP_ERROR_NULL_HANDLE:
        return "Null handle!";

      case MSP_ERROR_NO_ENOUGH_BUFFER:
        return "Buffer overflow!";

      default:
        return "Please refer to msp_errors.h or official development "
               "documentation for error codes.";
    }
  }
};
