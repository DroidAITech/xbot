/*
 * AIUIConstant.h
 *
 *  Created on: 2017年2月17日
 *      Author: hj
 */

#ifndef AIUICONSTANT_H_
#define AIUICONSTANT_H_

#include "AIUICommon.h"

namespace aiui {

class AIUIEXPORT AIUIConstant
{
public:
	/*******************事件类型**********************/
	/**
	 * 结果事件。info为结果描述JSON字符串，data为结果数据map，通过解析params
	 * 可以获取结果类型和结果的key值，然后用key值从data获取结果。
	 **/
	static const int EVENT_RESULT = 1;

	/**
	 * 错误事件。arg1为错误码，info为错误描述信息。
	 **/
	static const int EVENT_ERROR = 2;

	/**
	 * 状态事件。arg1为服务状态，取值：STATE_IDLE}、STATE_READY、STATE_WORKING。
	 **/
	static const int EVENT_STATE = 3;

	/**
	 * 唤醒事件。info为唤醒结果JSON字符串，示例：<br/>
	 * <pre>{
	 *   "power": 12342435436,   // 唤醒能量值
	 *   "beam":3,               // 拾音波束号，唤醒成功后阵列将在该波束方向上拾音
	 *   "angle":180,            // 唤醒角度
	 *   "channel":5,            // 唤醒声道，即麦克风编号，表示该声道的音频质量最好
	 *   "CMScore":132           // 声道对应的唤醒得分
	 * }</pre>
	 **/
	static const int EVENT_WAKEUP = 4;

	/**
	 * 休眠事件。当出现交互超时，或者外部发送CMD_RESET_WAKEUP消息重置唤醒时抛出该事件。
	 * arg1取值：TYPE_AUTO（自动休眠）、TYPE_COMPEL（外部强制休眠，发送CMD_RESET_WAKEUP）。
	 **/
	static const int EVENT_SLEEP = 5;

	/**
	 * VAD事件。arg1表示VAD消息类型，取值：VAD_BOS，VAD_VOL，VAD_EOS。
	 * 当arg1为VAD_VOL时，arg2表示音量，取值范围：[0-30]。
	 */
	static const int EVENT_VAD = 6;

	/**
	 * 绑定成功事件，当ServiceKit与服务绑定成功时抛出。Linux版本无此事件。
	 */
	static const int EVENT_BIND_SUCESS = 7;

	/**
	 * CMD对应的返回事件。arg1表示对应的CMD命令（如CMD_BUILD_GRAMMAR等），arg2为返回码（0表示成功），
	 * info为返回的结果。
	 */
	static const int EVENT_CMD_RETURN = 8;

	/**
	 * 音频抛出事件。16k音频数据在data中，通过键"audio"获取。
	 */
	static const int EVENT_AUDIO = 9;

	/**
	 * 准备休眠事件。抛出该事件通知外部即将休眠，若10秒之内无有效交互再抛出EVENT_SLEEP。
	 */
	static const int EVENT_PRE_SLEEP = 10;

	/**
	 * 开始录音事件。通知外部录音开始，用户可以开始说话。
	 */
	static const int EVENT_START_RECORD = 11;

	/**
	 * 停止录音事件。
	 */
	static const int EVENT_STOP_RECORD = 12;

	/**
	 * 与服务端建立起连接事件，建立连接后才能进行如数据同步等各种操作。
	 */
	static const int EVENT_CONNECTED_TO_SERVER = 13;

	/**
	 * 与服务端断开连接事件。
	 */
	static const int EVENT_SERVER_UNCONNECTED = 14;

	/**
	 * CAE鉴权抛出来的明文。
	 */
	static const int EVENT_CAE_PLAIN_TEXT = 1000;

	/**
	 * 推送消息事件。
	 */
	static const int EVENT_PUSH_MESSAGE = 1001;

	/*******************运行状态**********************/
	/**
	 * 空闲状态，AIUI服务未开启。
	 **/
	static const int STATE_IDLE = 1;

	/**
	 * 就绪状态，已经开启录音，等待唤醒。
	 **/
	static const int STATE_READY = 2;

	/**
	 * 工作状态，已经唤醒，可以开始人机交互。
	 **/
	static const int STATE_WORKING = 3;

	/*******************命令定义**********************/
	/**
	 * 获取AIUI服务状态。发送该命令后，AIUI会回应EVENT_STATE事件。
	 **/
	static const int CMD_GET_STATE = 1;

	/**
	 * 写入数据。params为描述数据信息（数据类型等）的JSON字符串，data携带二进制数据。
	 **/
	static const int CMD_WRITE = 2;

	/**
	 * 停止写入。params为描述数据信息（数据类型等）的JSON字符串。
	 **/
	static const int CMD_STOP_WRITE = 3;

	/**
	 * 重启服务。相当于停止后再启动服务，重启时会重新加载配置以使新配置生效。
	 **/
	static const int CMD_RESET = 4;

	/**
	 * 开启服务。用于在服务停止时启动服务，启动时会重新加载配置以使新配置生效。
	 **/
	static const int CMD_START = 5;

	/**
	 * 停止服务。用于在服务启动时停止服务，停止后不再接收输入。
	 **/
	static const int CMD_STOP = 6;

	/**
	 * 唤醒命令，用于外部主动唤醒。arg1字段为麦克风阵列波束编号，默认为0号波束，唤醒
	 * 后会向该波束方向拾音。唤醒成功后，会抛出EVENT_WAKEUP事件。
	 **/
	static const int CMD_WAKEUP = 7;

	/**
	 * 重置唤醒。将AIUI重置为待唤醒状态，由STATE_WORKING变为STATE_READY。
	 **/
	static const int CMD_RESET_WAKEUP = 8;

	/**
	 * 设置阵列拾音波束。arg1为波束编号，编号随阵列类型的不同而不同，设置后会向该波束方向拾音。
	 **/
	static const int CMD_SET_BEAM = 9;

	/**
	 * 设置参数。params为aiui.cfg文件内容，即一个JSON字符串。
	 **/
	static const int CMD_SET_PARAMS = 10;

	/**
	 * 上传用户词表。params为JSON字符串，示例：
	 * <pre>{
	 *   "name":"userword",   // 词表名称
	 *   "content":"XXXX"     // 词表内容
	 * }</pre>
	 * 其中XXXX也为一个JSON字符串，示例：
	 * <pre>{
	 *   "userword":[
	 *    {
	 *       "name":"我的常用词",
	 *       "words":["佳晨实业","蜀南庭苑","高兰路","复联二"]
	 *    },
	 *    {
	 *       "name":"我的好友",
	 *       "words":["李馨琪","鹿晓雷","张集栋","周家莉","叶震珂","熊泽萌"]
	 *    }]
	 * }</pre>
	 **/
	static const int CMD_UPLOAD_LEXICON = 11;

	/**
	 * 上传日志。params为JSON格式的日志。
	 **/
	static const int CMD_SEND_LOG = 12;

	/**
	 * 同步操作。
	 **/
	static const int CMD_SYNC = 13;

	/**
	 * 开始保存数据。params为键值对参数，用于指明数据类型，示例：
	 * data_type=raw_audio，暂时只支持原始音频一种类型的数据。
	 **/
	static const int CMD_START_SAVE = 14;

	/**
	 * 停止保存数据。params为键值对参数，用于指明数据类型，示例：
	 * data_type="raw_audio"，暂时只支持原始音频一种类型的数据。
	 **/
	static const int CMD_STOP_SAVE = 15;

	/**
	 * 构建识别语法。params为bnf语法内容。
	 **/
	static const int CMD_BUILD_GRAMMAR = 16;

	/**
	 * 更新本地词典，必须在调用过CMD_BUILD_GRAMMAR之后调用。
	 * params为JSON字符串，用于指定词表名称和内容，示例：<pre>
	 * {
	 *   "name":"<contact>",  // 词表名称
	 *   "content":"张三\n李四\n" //词表内容
	 * }<pre>
	 **/
	static const int CMD_UPDATE_LOCAL_LEXICON = 17;

	/**
	 * 开始抛出识别音频。调用之后，抛出EVENT_AUDIO事件。
	 * arg1为拾音波束编号设置，若当前未唤醒则会使用该波束拾音，若已经处于唤醒状态则arg1不起作用。
	 **/
	static const int CMD_START_THROW_AUDIO = 18;

	/**
	 * 停止抛出识别音频。调用之后，停止抛出EVENT_AUDIO事件。
	 **/
	static const int CMD_STOP_THROW_AUDIO = 19;

	/**
	 * 确认云端返回结果的有效性，用于延后休眠，在得到结果之后5秒时间内发送有效。
	 */
	static const int CMD_RESULT_VALIDATION_ACK = 20;

	/**
	 * 清除云端语义对话历史。
	 */
	static const int CMD_CLEAN_DIALOG_HISTORY = 21;

	/**
	 * 开始录制，需要在params中通过data_type字段指定录制的数据类型。
	 */
	static const int CMD_START_RECORD = 22;

	/**
	 * 停止录制，需要在params中通过data_type字段指定停止录制的数据类型。
	 */
	static const int CMD_STOP_RECORD = 23;

	/**
	 * 查询数据同步状态，需要在params中通过sid字段指定CMD_SYNC返回的sid。
	 */
	static const int CMD_QUERY_SYNC_STATUS = 24;

	/**
	 * CAE鉴权，写入设备信息。
	 */
	static const int CMD_CAE_WRITE_DEVINFO = 1000;

	/**
	 * CAE鉴权，写入加密后的密文。
	 */
	static const int CMD_CAE_WRITE_ENCRYPT = 1001;

	/*******************参数key值**********************/
	/**
	 * 应用唯一标识，即在<a href="http://www.xfyun.cn/">语音云平台</a>上申请的8位应用Id。
	 */
	static const char * const KEY_APPID;

	/**
	 * 连接的服务器地址。
	 */
	static const char * const KEY_SERVER_URL;

	/**
	 * AIUI数据上传地址。
	 */
	static const char * const KEY_AIUI_UP_URL;

	/**
	 * AIUI推送结点地址，即下发结果的服务器地址。
	 */
	static const char * const KEY_AIUI_PUSHNODE_URL;

	/**
	 * 获取chid的服务器地址。
	 */
	static const char * const KEY_AIUI_CHID_URL;

	/**
	 * 交互超时（单位：ms），即唤醒后一定时间无有效结果则休眠。
	 * 取值：[10000,180000)。
	 */
	static const char * const KEY_INTERACT_TIMEOUT;

	/**
	 * 网络超时时间（单位：ms）。
	 * {@hide}
	 */
	static const char * const KEY_NETWORK_TIMEOUT;

	/**
	 * 结果超时时间（单位：ms），即检测到音频后端点后一段时间内无结果则超时。默认值：5000。
	 */
	static const char * const KEY_RESULT_TIMEOUT;

	/**
	 * VAD开关，取值：0（关闭）、1（打开）。
	 */
	static const char * const KEY_VAD_ENABLE;

	/**
	 * VAD类型设置。取值：meta（模型VAD）、fix_front（能量VAD），默认值：meta。
	 */
	static const char * const KEY_VAD_TYPE;

	/**
	 * VAD前端点超时（单位：ms）。取值范围：[1000, 10000]，默认值：5000。
	 */
	static const char * const KEY_VAD_BOS;

	/**
	 * VAD后端点超时（单位：ms）。取值范围：[0, 10000]，默认值：1500.
	 */
	static const char * const KEY_VAD_EOS;

	/**
	 * 语种设置。暂不支持。
	 */
	static const char * const KEY_LANGUAGE;

	/**
	 * 方言设置。暂不支持。
	 */
	static const char * const KEY_ACCENT;

	/**
	 * 识别结果标点设置。暂不支持。
	 */
	static const char * const KEY_ASR_PTT;

	/**
	 * 引擎类型。
	 */
	static const char * const KEY_ENGINE_TYPE;

	/**
	 * 资源类型，取值：path（sdcard下的资源）、assets（assets下的资源）
	 */
	static const char * const KEY_RES_TYPE;

	/**
	 * 资源地址。
	 */
	static const char * const KEY_RES_PATH;

	/**
	 * 标记参数。
	 */
	static const char * const KEY_TAG;

	/**
	 * 业务场景。
	 */
	static const char * const KEY_SCENE;

	/**
	 * 数据类型，取值：text、raw_audio、audio、image、video。
	 */
	static const char * const KEY_DATA_TYPE;

	/**
	 * 数据来源，取值：user（外部写入）、sdk（SDK内部）。
	 */
	static const char * const KEY_DATA_SOURCE;

	/**
	 * 音频采样率（单位：Hz），取值：8000、16000。
	 */
	static const char * const KEY_SAMPLE_RATE;

	/**
	 * 语音识别用户个性化数据，可以提升指定词语的识别效果。
	 */
	static const char * const KEY_IAT_USER_DATA;

	/**
	 * 语义理解用户个性化数据，可以在语义业务中使用。
	 */
	static const char * const KEY_NLP_USER_DATA;

	/**
	 * 工作模式，每一种业务都有工作模式设置。<p>
	 * 语音业务的工作模式：rec_only（仅使用当麦克风阵列录音）、intent（意图模式，对音频进行处理，返回意图分析结果），默认值：intent。</p>
	 */
	static const char * const KEY_WORK_MODE;

	/**
	 * 唤醒模式，取值：ivw（一般唤醒）、cae（阵列唤醒）、off（关闭）。
	 */
	static const char * const KEY_WAKEUP_MODE;

	/**
	 * 处理意图的引擎类型，取值：local（离线处理，即本地语法识别）、cloud（云端处理，即语义）、mixed（混合方式）。
	 */
	static const char * const KEY_INTENT_ENGINE_TYPE;

	/*
	 * 交互模式。
	 */
	static const char * const KEY_INTERACT_MODE;

	/**
	 * 门限。
	 */
	static const char * const KEY_THRESHOLD;

	/**
	 * 名称。
	 */
	static const char * const KEY_NAME;

	/**
	 * 内容字段。
	 */
	static const char * const KEY_CONTENT;

	/**
	 * 构建路径。
	 */
	static const char * const KEY_BUILD_PATH;

	/**
	 * 语法id。
	 */
	static const char * const KEY_GRAMMAR_ID;

	/**
	 * 词表id。
	 */
	static const char * const KEY_LEXICON_ID;

	/**
	 * CAE库名。
	 */
	static const char * const KEY_LIB_CAE;

	/**
	 * CAE参数设置。
	 */
	static const char * const KEY_CAE_PARAMS;
	/*******************参数取值**********************/
	/**
	 * sdcard资源类型。
	 */
	static const char * const RES_TYPE_PATH;

	/**
	 * assets资源类型。
	 */
	static const char * const RES_TYPE_ASSETS;

	/**
	 * 在线引擎。
	 */
	static const char * const ENGINE_TYPE_CLOUD;

	/**
	 * 本地引擎。
	 */
	static const char * const ENGINE_TYPE_LOCAL;

	/**
	 * 同时使用在线、本地引擎。
	 */
	static const char * const ENGINE_TYPE_MIXED;

	/**
	 * 录音模式。
	 */
	static const char * const WORK_MODE_REC_ONLY;

	/**
	 * 意图模式。
	 */
	static const char * const WORK_MODE_INTENT;

	/**
	 * 一次交互，对于主意即“一次唤醒，一次交互”。
	 */
	static const char * const INTERACT_MODE_ONESHOT;

	/**
	 * 持续交互，对于语音即“一次唤醒，多次交互”。
	 */
	static const char * const INTERACT_MODE_CONTINUOUS;

	static const char * const AUTO;

	static const char * const USER;

	static const char * const KEY_CLEAN_DIALOG_HISTORY;

	/*******************alsa参数**********************/
	/**
	 * 声卡设备号，按实际情况填写。
	 */
	static const char * const KEY_SOUND_CARD;

	/**
	 * 声卡采样率（单位：Hz）。
	 */
	static const char * const KEY_CARD_SAMPLE_RATE;

	/*******************log参数**********************/
	/**
	 * debug日志开关，取值：0（关闭），1（打开）。
	 */
	static const char * const KEY_DEBUG_LOG;

	/**
	 * 是否保存data日志，取值：0（否），1（是）。
	 */
	static const char * const KEY_SAVE_DATALOG;

	/**
	 * data日志的保存目录，必须以"/"结尾。
	 */
	static const char * const KEY_DATALOG_PATH;

	/**
	 * 原始音频保存目录，必须以"/"结尾。
	 */
	static const char * const KEY_RAW_AUDIO_PATH;

	/*******************错误码定义**********************/
	/**
	 * 操作成功。
	 */
	static const int SUCCESS								= 0;
	/**
	 * 操作失败。
	 */
	static const int FAIL									= -1;

	/*******************其他常量**********************/
	/**
	 * VAD消息类型，前端点。
	 */
	static const int VAD_BOS = 0;

	/**
	 * VAD消息类型，音量。
	 */
	static const int VAD_VOL = 1;

	/**
	 * VAD消息类型，后端点。
	 */
	static const int VAD_EOS = 2;

	/**
	 * 由SDK内部自动抛出的事件。
	 */
	static const int TYPE_AUTO = 0;

	/**
	 * 由外部操作导致抛出的事件。
	 */
	static const int TYPE_COMPEL = 1;

	/**
	 * 同步状态数据。
	 */
	static const int SYNC_DATA_STATUS = 0;

	/**
	 * 同步个性化数据。
	 */
	static const int SYNC_DATA_INDIVIDUAL = 1;

	/**
	 * 第三方账号关联数据。
	 */
	static const int SYNC_DATA_ACCOUNT = 2;

	/**
	 * 同步雅典娜个性化数据。
	 */
	static const int SYNC_DATA_SCHEMA = 3;

	/**
	 * 查询数据同步状态。
	 */
	static const int SYNC_DATA_QUERY = 4;

	/**
	 * 所见即可说数据。
	 */
	static const int SYNC_DATA_SPEAKABLE = 5;

	static const char* const EMPTY_STRING;
};

}

#endif /* AIUICONSTANT_H_ */
