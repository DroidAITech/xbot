#!/usr/bin/env python
#coding=utf-8
"""
仿真时候的键盘控制

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy, sys, termios, tty
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class multi_keybroad_handle():


 def __init__(self):
  r=rospy.Rate(5)
  pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)


  start_time = time.time()
  count=0
  while not rospy.is_shutdown():
    if count<10:

      self.cmd = Twist()
      self.cmd.linear.x = 0
      self.cmd.angular.z = -0.9#.31415926*2.5
      pub.publish(self.cmd)
      print 'move forward'
      r.sleep()
    # elif (count>=10)&(count<20):

      self.cmd = Twist()
      self.cmd.linear.x = 0
      self.cmd.angular.z = 0.9#.31415926*2.5
      pub.publish(self.cmd)
      print "turn"
      r.sleep()
    else:
      self.cmd = Twist()
      pub.publish(self.cmd)
    count+=1







if __name__=='__main__':
 rospy.init_node('fake_keyboard_teleop')
 try:
  rospy.loginfo( "initialization system")
  multi_keybroad_handle()
  print "process done and quit"
 except rospy.ROSInterruptException:
  rospy.loginfo("node terminated.")

