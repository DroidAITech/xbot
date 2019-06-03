#/usr/bin/env python
#coding=utf-8
"""
仿真时候的键盘控制

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy, sys, termios, tty

from geometry_msgs.msg import Twist


class multi_keybroad_handle():
 def define(self):
  self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
  # self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

  self.notice = """
   Reading from the keyboard  and Publishing to Twist!
   ---------------------------
   robot Moving around:
                            i(x speed)
     h(angular speed)       j(y speed)    k    l(y speed) ;(angular speed)
                            ,(x speed)

   i: forward
   ,: backward
   j: walking left
   l: walking right
   h: turn left
   ;: turn right
   k: stop
   """

  self.robot_control = {
    'i':(1,0,0,0),
    'h':(0,0,0,1)
    ';':(0,0,0,-1)
    'j':(0,1,0,0),
    'l':(0,-1,0,0),
    'k':(0,0,0,0),
    ',':(-1,0,0,0),
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
  speed=0.3


  try:
   print self.notice
   status=0
   while not rospy.is_shutdown():
    self.cmd = Twist()

    key = self.getKey()

    if key in self.robot_control.keys():
     x = self.robot_control[key][0]*speed
     y = self.robot_control[key][1]*speed
     z = self.robot_control[key][2]*speed
     th = self.robot_control[key][3]*0.5
     self.cmd.linear.x = x
     self.cmd.linear.y = y
     self.cmd.angular.z = th
     if not status:
      print self.notice

    status = (status + 1) % 10
    if (key == '\x03'):
     break

    self.pub.publish(self.cmd)
    print self.cmd

    #rospy.sleep(0.1)
    #if self.cmd != Twist():
     #self.cmd = Twist()
     #self.pub.publish(self.cmd)

  except :
   print 'error'

  finally:
   self.pub.publish(self.cmd)

if __name__=='__main__':
 rospy.init_node('fake_keyboard_teleop')
 try:
  rospy.loginfo( "initialization system")
  multi_keybroad_handle()
  print "process done and quit"
 except rospy.ROSInterruptException:
  rospy.loginfo("node terminated.")

