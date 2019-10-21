# xbot_face

## 简介

xbot_face是用于重德智能XBot-U科研教学平台机器人的人脸识别ROS程序包。

该程序包具有人脸注册、人脸管理、人脸注销、人脸识别等多种功能。



## 使用方法
### 人脸注册

#### 1、机器人上注册

直接连接显示器在机器人上注册人脸的模式需要在未运行占用XBot-U机器人的人脸识别摄像头设备程序的情况下运行。

默认出厂XBot-U机器人上开机自动运行了人脸识别程序，占用了人脸识别摄像头，可通过在机器人上运行以下指令解除：

```
sudo service xbot stop
```

然后运行程序01face_register.py即可注册：

```
roscd xbot_face/script/
python 01face_register.py camera/image camera_index/name
```



#### 2、连入XBot-U网络的PC上注册

该模式使用的是PC的摄像头，直接运行01face_register.py程序即可。

```
roscd xbot_face/script/
python 01face_register.py camera/image camera_index/name 
```



### 人脸管理

在连接机器人网络的任意PC或机器人本体上打开链接<http://192.168.8.141:8000/management/userids>即可看到注册的人脸列表。



### 人脸注销

在连接机器人网络的任意PC或机器人本体上运行：

```
roscd xbot_face/script/
python 02face_logout.py userid/all
```

参数为注销某个用户id或者所有用户。



### 人脸识别

在机器人上运行：

```
roscore
rosrun xbot_face pub_camera_image
rosrun xbot_face face_recog.py
```

然后通过查看消息获得人脸识别结果：

```
rostopic echo /xbot/face_result
```



## 参考链接

- [ROSwiki xbot tutorials](<http://wiki.ros.org/Robots/Xbot/tutorial/cn>)
- [ROSwiki xbot_face软件说明](http://wiki.ros.org/xbot_face)
- [重德智能](https://www.droid.ac.cn/)
- [XBot-U机器人网站介绍](https://www.droid.ac.cn/xbot_u.html)
- [中国大学慕课-机器人操作系统入门](https://www.icourse163.org/course/0802ISCAS001-1002580008)

## 联系我们

**商务合作**：bd@droid.ac.cn

**技术咨询**：wangpeng@droid.ac.cn或添加微信:18046501051（注明XBot-U咨询）



