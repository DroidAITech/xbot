/**
 * AddTwoInts Client
 */
 
#include <cstdlib>
#include "ros/ros.h"
#include "xbot_talker/request_talker.h"

int main(int argc, char **argv)
{
  // ROS节点初始化
  ros::init(argc, argv, "request_talker_client");
  
  // 从终端命令行获取两个加数
  if (argc != 2)
  {
    ROS_INFO("usage: request_talker_client request");
    return 1;
  }

  // 创建节点句柄
  ros::NodeHandle n;
  
  // 创建一个client，请求add_two_int service，service消息类型是learning_communication::AddTwoInts
  ros::ServiceClient client = n.serviceClient<learning_communication::AddTwoInts>("request_talker");
  
  // 创建learning_communication::AddTwoInts类型的service消息
  learning_communication::request_talker srv;
  srv.request.request_talker = atoll(argv[1]);
  
  // 发布service请求，等待加法运算的应答结果
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (int)srv.response.response);
  }
  else
  {
    ROS_ERROR("Failed to call service request_talker");
    return 1;
  }

  return 0;
}
