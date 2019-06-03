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
 def define(self):
  self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 1)


  self.notice = """
   Reading from the keyboard  and Publishing to Twist!
   ---------------------------
   robot Moving around:
           i
      j    k    l
           ,

   i: forward
   ,: backward
   j: left turning
   l: right turning
   k: stop
   """

  self.robot_control = {
		'i':(1,0,0,0),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'k':(0,0,0,0),
		',':(-1,0,0,0),

		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'K':(0,0,0,0)
		}


 def getKey(self):

  tty.setraw(sys.stdin.fileno())
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
  return key


 def __init__(self):
  self.define()

  self.old_settings = termios.tcgetattr(sys.stdin)

  x = 0
  th = 0
  speed=0.2
  r=rospy.Rate(5)


  try:
   print self.notice
   status=0
   start_time = time.time()
   while not rospy.is_shutdown():


    self.cmd = Twist()

    self.cmd.linear.x = 0
    self.cmd.angular.z = 3.1415926*1.9/180
    self.pub.publish(self.cmd)
    r.sleep()

      # self.cmd.linear.x = 0
      # self.cmd.angular.z = 0#.31415926*5
      # self.pub.publish(self.cmd)
      # r.sleep()
    # else:
    #   self.cmd.linear.x = 0
    #   self.cmd.angular.z = 0#.31415926*5
    #   self.pub.publish(self.cmd)
    # elif (time.time()-start_time)<=30:
    #   self.cmd.linear.x = -0.2
    #   self.cmd.angular.z = 0
    #   self.pub.publish(self.cmd)
    # elif (time.time()-start_time)<=40:
    #   self.cmd.linear.x = 0.2
    #   self.cmd.angular.z = 0#.31415926*5
    #   self.pub.publish(self.cmd)



  except :
   print 'error'

  finally:
   self.pub.publishf(self.cmd)





if __name__=='__main__':
 rospy.init_node('fake_keyboard_teleop')
 try:
  rospy.loginfo( "initialization system")
  multi_keybroad_handle()
  print "process done and quit"
 except rospy.ROSInterruptException:
  rospy.loginfo("node terminated.")

