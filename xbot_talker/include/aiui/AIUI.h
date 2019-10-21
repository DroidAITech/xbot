/*
 * 	AIUI.h
 *
 *  Created on: 2017年2月17日
 *      Author: hj
 */

#ifndef AIUI_API_H_
#define AIUI_API_H_

#include "AIUIConstant.h"
#include "AIUIErrorCode.h"
#include "AIUIType.h"
#include "AIUICommon.h"

namespace aiui {

/**
 * 获取版本信息。
 */

const char* getVersion();

/**
 * AIUI事件类，业务结果、SDK内部状态变化等都通过事件抛出。
 * 回调后事件内存即可能立即释放。如需留用，做拷贝。
 */
class IAIUIEvent
{
public:
	AIUIEXPORT virtual ~IAIUIEvent();
	/**
	 * 事件类型，具体取值参见AIUIConstant。
	 */
	virtual int getEventType() = 0;

	/**
	 * 扩展参数1。
	 */
	virtual int getArg1() = 0;

	/**
	 * 扩展参数2。
	 */
	virtual int getArg2() = 0;

	/**
	 * 描述信息。
     * 返回的内存不可外部直接 delete。
	 */
	virtual const char* getInfo() = 0;

	/**
	 * 附带数据。
     * 返回的内存不可外部直接 delete。
	 */
	virtual IDataBundle* getData() = 0;
};

class IAIUIMessage
{
public:
	AIUIEXPORT virtual ~IAIUIMessage();

	/**
	 * 创建消息对象
     * @param msgType 消息类型 参见AIUIConstant::CMD_开头的定义
	 * @param arg1 参数1 参见http://www.xfyun.cn/中AIUI的技术文档
	 * @param arg2 参数2 参见http://www.xfyun.cn/中AIUI的技术文档
	 * @param params 业务参数。 传入后内部会做copy。取值参见http://www.xfyun.cn/中AIUI的技术文档
     * @param data 业务数据。不会做拷贝. 参见http://www.xfyun.cn/中AIUI的技术文档
	 *             在Message在内部处理后会自动release, 不能在外部释放掉。
	 * @return IAIUIMessage对象指针
	 */
	AIUIEXPORT static IAIUIMessage*  create(
		int msgType ,
		int arg1 = 0,
		int arg2 = 0,
		const char* params = "",
		Buffer* data = 0);

	/**
	 * 消息类型，具体取值参见AIUIConstant。
	 */
	virtual int getMsgType() = 0;

	/**
	 * 扩展参数1。
	 */
	virtual int getArg1() = 0;

	/**
	 * 扩展参数2。
	 */
	virtual int getArg2() = 0;

	/**
	 * 业务参数。
	 */
	virtual const char* getParams() = 0;

	/**
	 * 附带数据。
	 */
	virtual Buffer* getData() = 0;

	/**
	 * 释放附带数据
	 */
	virtual void releaseData() = 0;

	/**
	 * 仅销毁自身，注意不会释放Buffer* data;
	 */
	virtual void destroy() = 0;
};

/**
 * AIUI监听接口。需要使用者实现
 */
class  AIUIListener
{
public:
	AIUIEXPORT virtual ~AIUIListener();
	/**
	 * 事件回调，SDK所有输出都通过event抛出。
	 *
	 * @param event AIUI事件
	 */
	virtual void onEvent( IAIUIEvent& event) = 0;
};
typedef AIUIListener IAIUIListener;

/**
 * AIUI代理，单例对象，应用通过代理与AIUI交互。
 */
class IAIUIAgent
{
public:
	AIUIEXPORT virtual ~IAIUIAgent();
	/**
	 * 创建Agent单例对象，AIUI开始工作。
	 * 注：该方法总是返回非空对象，非空并不代表创建过程中无错误发生。
	 *
	 * @param params 参数设置
	 * @param listener 监听器
	 * @return AIUIAgent单例对象
	 */
	AIUIEXPORT static IAIUIAgent* createAgent(const char* params, IAIUIListener* listener);

	/**
	 * 发送消息给AIUI，消息中可以包含命令、参数和数据，具体格式参见AIUIMessage。
	 *
	 * @param msg AIUI消息
	 * message 如果指定了非空的Buffer *data， 在Message在内部处理后会自动release()这部分data;
	 * 而不能外部去释放掉。
	 */
	virtual void sendMessage(IAIUIMessage* message) = 0;

	/**
	 * 销毁AIUIAgent对象，AIUI停止工作。
	 */
	virtual void destroy() = 0;
};

/**
 * 日志级别。
 */
enum LogLevel
{
	info,
	debug,
	error
};

/**
 * 版本类型。
 */
enum VersionType
{
	INTELLIGENT_HDW,	// 智能硬件版本
	MOBILE_PHONE		// 移动互联网版本
};

/**
 * AIUI设置类，用于日志等设置。
 */
class  AIUISetting
{
public:
	/**
	 * 设置AIUI文件夹路径，SDK会在该路径下保存日志等文件。
	 *
	 * @param dir 路径，以"/"结尾，不能为空
	 */
	AIUIEXPORT static void setAIUIDir(const char*  dir);

	/**
	 * 初始化日志记录器，只有初始化之后才会保存日志。
	 *
	 * @param logDir 日志目录，以"/"结尾，为空则在AIUIDir下创建log目录
	 */
	AIUIEXPORT static void initLogger(const char* logDir = "");

	/**
	 * 设置日志打印级别，默认级别为info。
	 */
	AIUIEXPORT static void setLogLevel(LogLevel level);

	/**
	 * 设置是否保存数据日志，即输入的音频和云端返回的结果。
	 */
	AIUIEXPORT static void setSaveDataLog(bool save);

	/**
	 * 设置数据日志保存目录，默认是在AIUIDir下的data目录下。
	 */
	AIUIEXPORT static void setDataLogDir(const char* dir);

	/**
	 * 设置原始音频保存目录，默认是在AIUIDir下的audio/raw/目录下。
	 */
	AIUIEXPORT static void setRawAudioDir(const char* dir);

	/**
	 * 设置设备的uniqueId。
	 */
	AIUIEXPORT static void setUniqueId(const char* uniqueId);

	/**
	 * 设置版本类型。
	 */
	AIUIEXPORT static void setVersionType(VersionType type);

	/**
	 * 是否为移动互联网版本。
	 */
	AIUIEXPORT static bool isMobileVersion();

	/**
	 * 获取版本类型。
	 */
	AIUIEXPORT static VersionType getVersionType();
};

/**
 * 工具类，用于MSC登录，退出，设置和获取参数。
 */
class ISpeechUtility
{
public:
	AIUIEXPORT virtual ~ISpeechUtility();
	/** 
	 * 创建单例对象
	 * @param usr 用户名
	 * @param pwd 密码
	 * @param params 业务参数, 包括appid和key等。比如 "appid=<appid>,key=<keystring>"
	 *               <appid>,<keystring>替换为实际的值
	 */
	AIUIEXPORT static ISpeechUtility* createSingleInstance(const char* usr, const char* pwd, const char* params);
	
	/**
	 * 获取实例
	 */
	AIUIEXPORT static ISpeechUtility* getInstance();

	/**
	 * 销毁
	 */
	virtual void destroy() = 0;

	virtual bool setParameter(const char* key, const char* val) = 0;

	virtual const char* getParameter(const char* key) = 0;
};


}

#ifdef AIUI_LIB_COMPILING
#include "aiui_internal/AIUI_internal.h"
#endif


#endif /* AIUI_API_H_ */
