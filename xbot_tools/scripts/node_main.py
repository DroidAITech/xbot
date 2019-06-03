#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""real-time reg 不用存.wav文件，直接识别文件的话语发布服务
Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot."""

import rospy
from std_msgs.msg import String

class recoder_main():
 def __init__(self):
  self.define()
  rospy.Subscriber(self.topic , String , self.talker)
  rospy.spin()
   
 def define(self):
  self.topic="stop_flag"
  self.say=rospy.Publisher('speak_string', String, queue_size=1)
  if rospy.has_param('~words'):
   pass
  else:
   rospy.set_param('~words','请让一下，谢谢')
  self.words=rospy.get_param('~words')
  
 def talker(self,data):
  if data.data=='stop':
   self.say.publish(self.words)
  else:
   pass
   
if __name__=="__main__":
 rospy.init_node('warning2speaker')
 rospy.loginfo("initialization system")
 recoder_main()
 rospy.loginfo("process done and quit")
