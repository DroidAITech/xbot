#!/usr/bin/env python
#coding=utf-8
"""
multi-goal tasks

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy
import getpass
import actions_reference
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose


class cruise_modle():
 def __init__(self):
  Plist=[]
  Current_position = rospy.wait_for_message("turtlebot_position_in_map", Pose)
  Plist.append(Current_position.position)
  #rospy.sleep(1)
  self.loading(Plist)
  
 def loading(self, Plist):
  data=raw_input('请输入巡航的目标点数量： ')
  try:
   num = int(data)
   for i in range(num):
    rospy.loginfo ('预备录入第%s个目标点'%(i+1))
    position =  rospy.wait_for_message("clicked_point",PointStamped)
    rospy.loginfo ('已录入第%s个目标点'%(i+1))
    Plist.append(position.point)
   
   actions_reference.cruise(Plist)
   
  except:
   self.loading(Plist)


if __name__ == '__main__':
 rospy.init_node('cruise_tasks')
 try:
  rospy.loginfo( "initialization system")
  cruise_modle()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("follower node terminated.")

