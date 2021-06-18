# xbot各功能包ROS接口

本章着重介绍在xbot功能包合集中的各程序包的ROS接口，便于用户调用、查看以及程序集成。

## 各ROS包主要功能

该软件包集包含7个ROS软件子包。

| ROS软件包              | 主要功能                                 |
| ---------------------- | ---------------------------------------- |
| xbot_bringup           | 驱动程序启动入口，包含多个launch文件     |
| xbot_driver            | 底层通讯驱动，收发数据                   |
| xbot_node              | ROS数据封装与分发，运动解算              |
| xbot_msgs              | 软件包集所用到的所有ROS消息类型          |
| xbot_safety_controller | 机器人安全控制程序                       |
| xbot_description       | ROS建模与urdf描述文件                    |
| xbot_tools             | 用于调试、开发以及运行过程的一些工具软件 |
| xbot_face              | 人脸识别程序                             |
| xbot_navi              | SLAM、导航规划程序                       |
| xbot_talker            | 机器人语音交互和对话程序                 |



## xbot_node机器人ROS驱动程序包

### 发布的话题

1. /mobile_base/joint_states (<sensor_msgs::JointState>)

   机器人关节状态

2. /mobile_base/sensors/core(<xbot_msgs::CoreSensor>)

   核心传感器数据

3. /mobile_base/sensors/extra(<xbot_msgs::ExtraSensor>)

   传感器数据

4. /mobile_base/sensors/yaw_platform_degree(<std_msgs::Int8>)

   水平云台角度

5. /mobile_base/sensors/pitch_platform_degree(<std_msgs::Int8>)

   竖直云台角度

6. /mobile_base/sensors/motor_enabled(<std_msgs::Bool>)

   轮子电机是否使能了

7. /mobile_base/sensors/sound_enabled(<std_msgs::Bool>)

   声音开关是否打开了

8. /mobile_base/snesors/battery(<xbot_msgs::Battery>)

   电池电量

9. /mobile_base/sensors/front_echo(<sensor_msgs::Range>)

   前方超声数据

10. /mobile_base/sensors/rear_echo(<sensor_msgs::Range>)

    后方超声数据

11. /mobile_base/sensors/infrared(<xbot_msgs::InfraRed>)

    红外数据

12. /imu(<sensor_msgs::Imu>)

    IMU数据

13. /odom(<nav_msgs::Odometry>)

    里程计数据

14. /tf(<geometry_msgs::TransformStamped>)

    坐标转换

15. /mobile_base/sensors/raw_imu_data(<xbot_msgs::RawImu>)

    原始九轴IMU数据

16. /mobile_base/xbot/state(<xbot_msgs::XbotState>)

    机器人状态

### 订阅的话题

1. /mobile_base/commands/motor_enable(<std_msgs::Bool>)

   控制电机

2. /mobile_base/commands/velocity(<geometry_msgs::Twist>)

   控制速度

3. /mobile_base/commands/yaw_platform(<std_msgs::Int8>)[-90~90]

   控制水平云台

4. /mobile_base/commands/pitch_platform(<std_msgs::Int8>)[-60~30]

   控制竖直云台

5. /mobile_base/commands/sound_enable(<std_msgs::Bool>)

   打开声音开关

6. /mobile_base/commands/led(<std_msgs::UInt8>)

   控制led灯。

7. /mobile_base/commands/reset_odometry(<std_msgs::Empty>)

   重置里程计

## xbot_face人脸识别程序包

### 发布的话题

1. /xbot/face_result(<xbot_face::FaceResult>)

   人脸识别的结果。

### 订阅的话题

1. /xbot/camera/image(<sensor_msgs::Image>)

   人脸识别摄像头的画面。

   

## xbot_talker语音交互程序包

### 发布的话题

1. /cmd_vel_mux/input/teleop(<geometry_msgs::Twist>)

   控制机器人运动

2. /xbot/talker_state(<xbot_talker::talk_monitor>)

   talker状态

3. /welcome/leave(<std_msgs::Bool>)

   前往触发信号

### 订阅的话题

None

### 提供的服务

1. /xbot/play(<xbot_talker::play>)

   语音播放

2. /xbot/chat(<xbot_talker::chat>)

   语音对话