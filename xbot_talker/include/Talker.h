#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include <string.h>
#include <signal.h>
#include "rapidjson/document.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>

#define ALSA_MAX_BUF_SIZE 65535



/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
        char            riff[4];                // = "RIFF"
        int		size_8;                 // = FileSize - 8
        char            wave[4];                // = "WAVE"
        char            fmt[4];                 // = "fmt "
        int		fmt_size;		// = 下一个结构体的大小 : 16
        short int       format_tag;             // = PCM : 1
        short int       channels;               // = 通道数 : 1
        int		samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
        int		avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
        short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
        short int       bits_per_sample;        // = 量化比特数: 8 | 16
        char            data[4];                // = "data";
        int		data_size;              // = 纯数据长度 : FileSize - 44
} wave_pcm_hdr;


///* 默认wav音频头部数据 */
static wave_pcm_hdr default_wav_hdr =
{
        { 'R', 'I', 'F', 'F' },
        0,
        {'W', 'A', 'V', 'E'},
        {'f', 'm', 't', ' '},
        16,
        1,
        1,
        44100,
        88200,
        2,
        16,
        {'d', 'a', 't', 'a'},
        0
};

	/*
	* rdn:           合成音频数字发音方式
	* volume:        合成音频的音量
	* pitch:         合成音频的音调
	* speed:         合成音频对应的语速
	* voice_name:    合成发音人
	* sample_rate:   合成音频采样率
	* text_encoding: 合成文本编码格式
	*/
static char session_tts_begin_params[200]  = "voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 100, pitch = 50, rdn = 2";


static char session_record_begin_params[200] =
                "sub = iat, domain = iat, language = zh_cn, "
                "accent = mandarin, sample_rate = 16000, "
                "result_type = plain, result_encoding = utf8";

//int 代表播放音频的请求码，string代表检索到的用户姓名
typedef void  (*on_play_finished)(int , std::string);

//简单的播放一个文件
static const int REQUEST_SIMPLE_PLAY = 1;

//对员工进行问候
static const int REQUEST_GREET_STAFF = 2;

//对访客进行问候
static const int REQUEST_GREET_VISITOR = 3;

//查询目标点(表示前往员工工位或者前往办公室)
static const int REQUEST_CHAT_QUERY_GOAL = 4;

//进行AI对话
static const int REQUEST_CHAT = 5;

//当到达了目标位置(员工工位)后，触发此事件
static const int REQUEST_REACH_GOAL = 6;

//当中途被挡住时
static const int REQUEST_MOVE_ABORT = 7;

//没有在语音库中匹配到恰当的回答语(确实不会回答或者是噪声引起的误识别)
static const int REQUEST_AUDIO_UNMATCH = 8;

//播放完hard.wav
static const int REQUEST_VERIFY_COMPLETE = 9;

//语音控制Xbot进行移动
static const int REQUEST_MOVE = 10;

static const int REQUEST_GREET_VIP = 11;
//语音控制关闭对话
static const int REQUEST_END_CHAT = 12;
static const int REQUEST_CHAT_UNMATCH = 13;

static const int REQUEST_LEAVE = 14;
static void simple_call_back(int code,std::string message){

}
class Talker {
	
private:
	rapidjson::Document dictionaryDoc;
	rapidjson::Document greetingDoc;
	std::string basePath;//根CMakeLists所在的目录
	std::string goalName;
	int uploadHotWords();
	
public: 
	Talker();
	~Talker();
	int init(std::string basepath);
	int greetByName(std::string name,on_play_finished callback);
	int chat(std::string  chatMsg,on_play_finished callback);
  int play(char* file,int requestcode,on_play_finished  callback);
	int text_to_speech(const char* src_text,const char* audioFile,int requestCode,on_play_finished callback);
};

