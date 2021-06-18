# xbot_talker

## 简介

xbot_talker是用于重德智能XBot-U科研教学平台机器人的**语音交互**ROS程序包。

该程序包提供了百度在线语音识别、科大讯飞离线唤醒、科大讯飞语音识别、科大讯飞离线命令词识别、科大讯飞离线语音合成以及机器人控制、图灵对话机器人模块的调用；以及完整的智能语音交互系统，能够通过语音控制机器人动作、导航以及对话聊天等功能。



## 安装依赖

下载功能包后请先连通网络，通过以下命令安装依赖：

```
chmod +x bootstrap.sh && ./bootstrap.sh && source ~/.profile
```


## xbot_talker.launch使用说明

xbot_talker.launch启动文件是xbot_talker智能语音交互系统的启动文件，集成了离线唤醒、语音识别、语音合成、nlp处理、机器人控制、app配置接口等功能。

第一次启动前需先构建语法：

```
roslaunch xbot_talker build_grammar.launch 
```

之后便可直接启动xbot_talker.launch文件进行测试：

```
roslaunch xbot_talker xbot_talker.launch 
```


说明：
- 1.`！！！注意！！！`:修改asr_module.launch里的参数可以选择不同语音识别功能，考虑到真机的交互主次关系以及机器人准确动作的成功率，现仅提供`！三种！`参数设置模式:（1）纯离线模式，enable_offline设置为`true`,其余设置为`false`；(2)离线+科大讯飞在线，enable_offline设置为`true`,enable_xfei_online设为`true`,enable_baidu_online设置为`false`；(3)离线+百度在线，enable_offline设置为`true`,enable_xfei_online设为`false`,enable_baidu_online设置为`true`。

- 2.启动xbot_talker.launch后会进入唤醒模块，等待唤醒，可通过唤醒词“小德小德”唤醒机器人智能语音交互功能，成功唤醒后会听到"我在"的提示音，提示音结束后可以通过语音与机器人进行交互。

- 3.若离线识别结果置信度较低，或没有人说话，会播放“识别不够准确，请再说一遍命令词吧”等的提示，提示后用户可以再说一遍关键词，或者进入唤醒状态等待用户再次唤醒机器人

- 4.命令词“开始对话”、“开启对话模式”、“开启多轮交互模式”“陪我聊天”等命令词可以使机器人进入多轮交互模式。用户可以在机器人响应结束后开始新一轮语音交互，不需要再通过唤醒词进行唤醒。通过命令词“关闭”、“退出对话”、“退出聊天”“关闭多轮交互模式”等命令词退出多轮交互模式，之后会自动进入`等待唤醒状态`。

- 5.导航功能功能相关命令词，例如"回到原点"、“去某某办公室”等，响应后语音交互模块会自动进入等待唤醒状态。

- 6.除了语音交互，还能进行语音合成、app配置对话库等内容，更多测试和说明请参考后面各个模块的介绍。


## build_grammar节点
构建离线命令词语法识别节点，第一次启用asr（离线命令词识别）节点前需要先运行此节点构建语法，否则会提示找不到愈发文件。

- 构建语法：`roslaunch xbot_talker build_grammar.launch `

一次构建语法完成后，只要不修改bnf语法文件，再启用asr节点前便无需启动此节点。

## asr_sample节点

提供离线关键词识别功能。通过修改参数（修改launch文件中对应的参数）可实现以下功能：

- 1.读取单声道pcm音频文件进行语法识别并输出识别结果；
- 2.读取双声道pcm音频文件进行语法识别并输出识别结果；
- 3.声卡设备实时录音进行语法识别并输出识别结果（默认3.5s，可修改录音时长）； 
- 4.USB_MIC实时录音进行语法识别并输出识别结果；（`TODO`）

launch文件：

- 启动asr节点：`roslaunch xbot_talker asr_module.launch `

### 参数Parameters
- asr_mode:语法识别模式。`mic`:asr模块通过mic实时录音进行语音识别;`file`:读取pcm文件数据进行语音识别。
- base_path:到xbot_talker的绝对路径。
- grammar_path: 保存生成的语法文件的绝对路径。
- pcm_file:待读取的pcm音频文件。
- audio_channel:音频数据声道数目。

### 订阅话题Subscribed Topics
- /xbot_talker/awaken_status(awaken_status.msg):从此话题订阅语音唤醒的结果。


### 发布话题Published Topics

- /xbot_talker/recog_result(recog_result.msg):从此话题发布科大讯飞离线语法识别的结果和准确度。

- /xbot_talker/online_recog_result(online_asr_result.msg):从此话题发布科大讯飞或者百度在线语音识别的结果。

- /mobile_base/commands/sound_enable(std_msgs/Bool):控制机器人喇叭是否开启，需要在每次录音前关闭喇叭，录音结束后开启喇叭，提高录音效果。


### service服务

- /xbot_talker/asr_keyword_config(keyword_config.srv):提供重新加载bnf语法参数的服务。

## tts_sample节点

- 启动tts节点：`roslaunch xbot_talker tts_module.launch`

- 测试xbot_tts服务：`rosservice call /xbot/xbot_tts True "重德智能欢迎您"`

## awaken_sample节点

提供语音唤醒功能。通过修改参数（修改launch文件中对应的参数）可实现以下功能：

- 1.读取单声道pcm音频文件进行语音唤醒测试并输出唤醒结果；
- 2.读取双声道pcm音频文件进行语音唤醒测试并输出唤醒结果；
- 3.声卡设备实时录音进行语音唤醒并输出唤醒结果（可修改录音开启时长）； 
- 4.USB_MIC实时录音进行语音唤醒并输出唤醒结果；（`TODO`）

launch文件：

- roslaunch xbot_talker awaken_module.launch 


### 订阅话题Subscribed Topics
- /xbot_talker/enable_awake(std_msgs/Bool):当msg为true时，开始语音唤醒。

### 发布话题Published Topics

- /xbot_talker/awaken_status(awaken_status.msg):从此话题发布语音唤醒的结果。

- /mobile_base/commands/sound_enable(std_msgs/Bool):控制机器人喇叭是否开启，需要在每次录音前关闭喇叭，录音结束后开启喇叭，提高录音效果。



## nlp_sample节点
离线命令词识别的处理模块，包括检测识别结果是否存在、进行响应、控制机器人移动、控制机械臂等功能。

- 启动nlp节点：`roslaunch xbot_talker nlp_module.launch`

### 订阅话题Subscribed Topics
- /xbot_talker/recog_result(recog_result.msg):从此话题订阅科大讯飞离线语法识别的结果和准确度。

- /xbot_talker/online_recog_result(online_asr_result.msg):从此话题订阅科大讯飞或者百度在线语音识别的结果。


### 发布话题Published Topics

- /xbot_talker/awaken_status(awaken_status.msg):从此话题发布语音唤醒的结果。nlp模块在enable_chat模式下，会向此话题发布消息开启下一轮离线命令词识别。
- /xbot_talker/enable_awake(std_msgs::Bool):从此话题发布true的消息进入语音唤醒模式。
- /cmd_vel_mux/input/teleop(geometry_msgs::Twist):从此话题获取机器人运动的速度信息。
- /arm/commands/left_get(std_msgs::Empty):从此话题获取抬起机械臂左手的信息。
- /arm/commands/left_put(std_msgs::Empty):从此话题获取放下机械臂左手的信息。
- /arm/commands/right_get(std_msgs::Empty):从此话题获取抬起机械臂右手的信息。
- /arm/commands/right_put(std_msgs::Empty):从此话题获取放下机械臂右手的信息。
- /arm/commands/left_grip(std_msgs::Bool):从此话题获取机械臂左手爪抓/合的信息。
- /arm/commands/right_grip(std_msgs::Bool):从此话题获取机械臂右手爪抓/合的信息。
- /welcome/yes(std_msgs::Bool):从此话题发布开始导航控制
- /welcome/kp(std_msgs::String)：从此话题发布需要导航至的目标点（导航配置）

### service服务

- /xbot_talker/nlp_dialog_config(nlpdialog_config.srv):提供添加对话策略到csv对话库并重新加载文件的服务。

## dialogue_config节点

**接口说明**

ROS端运行

```
roslaunch rosbridge_server rosbridge_websocket.launch 
rosrun xbot_talker dialogue_configuration.py
```

**１、获取行为字典信息**

- service: `/app/get_action`
- request: `bool get_action_req`
- response: `xbot_talker/action_ros action_list`

APP发送请求true可以获取行为字典信息：

![action](/doc/image/action.png)

**2、获取动作模式字典信息**

- service: `/app/get_actionmode`

- request: `bool get_actionmode_req`

- response: `xbot_talker/actionmode_ros actionmode_list`

APP发送请求true可以获取动作模式字典信息：

![actionmode](/doc/image/actionmode.png)

**3、保存对话配置并部署**

* service：`/app/add_dialogue`
* request: `string keyword` `string answer` `uint8 action`  `uint8 action_mode`
* response: `uint8 config_status`（返回值说明：０表示成功，１表示关键词已存在，２表示bnf语法错误）

APP发送请求，如

```
keyword: '鼠标'
answer: '键盘'
action: 0
action_mode: 3
```

`注意：`action和action_mode的编码号分别对应不同的含义，参考xbot_talker/userconfig/json文件夹下action.json、action_mode.json

ROS端显示：

![save dialogue](/doc/image/save dialogue.png)

**配置成功之后的验证**

以上３个接口实现之后可通过以下方式验证

1、ROS端运行

```shell
roslaunch rosbridge_server rosbridge_websocket.launch
roslaunch xbot_talker xbot_talker.launch 
```

**语音识别操作说明：**

- 说出唤醒词“小德小德“进入语音识别状态，此时说出已有的关键词，比如”你好“，会收到应答”你好，我是机器人小德“
- 识别成功之后进入下一次对话需要再次进行唤醒
- 识别不成功会提醒你重新输入关键词

２、APP通过对话配置添加新的对话，唤醒”小德小德“之后说出新添加的关键词keyword，即可得到最新配置的对话内容

**4、查看对话配置历史记录**

* service:`/app/history_record`
* request:`bool get_history_record`
* response:`xbot_talker/dialogue_ros dialogue_list`

APP发送请求true可以获取最近添加的 10 条历史记录(下图为部分截图)：

![](doc/image/history_record.png)

**5、回滚功能**

* service:`/app/roolback_dialogue`
* request:`string version_info` （请求说明：这里取对话配置项中`update_time`作为回滚标记）
* response:`uint8 config_status`（返回值说明：0表示成功，1表示失败）

APP发送请求信息如`2019-11-26 21:02:02`，取自如下对话配置：

```
"answer": "苹果", 
"action": 0, 
"update_time": "2019-11-26 21:02:02", 
"action_mode": 3, 
"keyword": "香蕉"
```

对话配置库文件将回滚到本条对话配置之前的设定（包括本条），ROS端日志显示：

![](doc/image/roolback_succeed.png)

如果未成功，将显示：

![](doc/image/roolback_failed.png)

## 参考链接

- [ROSwiki xbot tutorials](<http://wiki.ros.org/Robots/Xbot/tutorial/cn>)
- [ROSwiki xbot_talker软件说明](http://wiki.ros.org/xbot_talker)
- [重德智能](https://www.droid.ac.cn/)
- [XBot-U机器人网站介绍](https://www.droid.ac.cn/xbot_u.html)
- [中国大学慕课-机器人操作系统入门](https://www.icourse163.org/course/0802ISCAS001-1002580008)

## 联系我们

**商务合作**：bd@droid.ac.cn

**技术咨询**：wangpeng@droid.ac.cn或添加微信:18046501051（注明XBot-U咨询）



