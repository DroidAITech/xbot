# xbot_talker

## 简介

xbot_talker是用于重德智能XBot-U科研教学平台机器人的**语音交互**ROS程序包。

该程序包具有离线命令词识别、离线TTS以及离线对话等多种功能。



## 安装依赖

安装命令行播放音乐工具：

```
sudo apt install sox
sudo apt install libsox-fmt-all
```

官方开发文档：<https://doc.xfyun.cn/msc_linux/>



## 使用方法

在机器人上运行

```
roslaunch xbot_talker talker.launch
```



### 调用播放服务

#### 播放文字合成语音

在ROS系统上运行

```
rosservice call /xbot/play "loop: false
mode: 2
audio_path: ''
tts_text: '你好'" 
```

#### 播放制定文件

在ROS系统上运行

```
rosservice call /xbot/play "loop: false
mode: 1
audio_path: 'music.wav'
tts_text: ''" 
```



### 调用对话服务

在ROS系统上运行：

```
rosservice call /xbot/chat "start_chat: true"
```



### 查看talker当前状态

```
rostopic echo /xbot/talk_state
```



## 参考链接

- [ROSwiki xbot tutorials](<http://wiki.ros.org/Robots/Xbot/tutorial/cn>)
- [ROSwiki xbot_talker软件说明](http://wiki.ros.org/xbot_talker)
- [重德智能](https://www.droid.ac.cn/)
- [XBot-U机器人网站介绍](https://www.droid.ac.cn/xbot_u.html)
- [中国大学慕课-机器人操作系统入门](https://www.icourse163.org/course/0802ISCAS001-1002580008)

## 联系我们

**商务合作**：bd@droid.ac.cn

**技术咨询**：wangpeng@droid.ac.cn或添加微信:18046501051（注明XBot-U咨询）



