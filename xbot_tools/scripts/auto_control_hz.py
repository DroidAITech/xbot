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
  pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
  x = 0
  th = 0
  speed=0.2
  r=rospy.Rate(17)



  start_time = time.time()
  count=0
  while not rospy.is_shutdown():
    if count<10:

      self.cmd = Twist()
      self.cmd.linear.x = 0.1
      self.cmd.angular.z = 0#.31415926*2.5
      pub.publish(self.cmd)
      print 'move forward'
    elif (count>=10)&(count<20):

      self.cmd = Twist()
      self.cmd.linear.x = 0
      self.cmd.angular.z = -0.3#.31415926*2.5
      pub.publish(self.cmd)
      print "turn"
    else:
      self.cmd = Twist()
      pub.publish(self.cmd)

    count+=1
    r.sleep()


  # if (time.time()-start_time)<=5:
  #   self.cmd.linear.x = 0
  #   self.cmd.angular.z = 0.8#.31415926*2.5
  #   self.pub.publish(self.cmd)
  #   r.sleep()








if __name__=='__main__':
 rospy.init_node('fake_keyboard_teleop')
 try:
  rospy.loginfo( "initialization system")
  multi_keybroad_handle()
  print "process done and quit"
 except rospy.ROSInterruptException:
  rospy.loginfo("node terminated.")

