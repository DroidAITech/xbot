#!/usr/bin/env python
#coding=utf-8
"""
this code used to set robot init pose by keyboard
Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""

import rospy, sys, termios, tty, numpy, PyKDL
from geometry_msgs.msg import PoseWithCovarianceStamped

class init_pose_handle():

 def define(self):
  rospy.init_node('keyboard_init_pose')
  self.position_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 1)
  self.terminal_msg="""
  
   Using following keys to control robot_init_pose
   ------------------------------
                up     
     right    space    left
               down
     PageUp            PageDown
              Ctrl+s   
   ------------------------------
   right:    moving robot position right (m)
   left:     moving robot position left  (m)
   up:       moving robot position up    (m)
   down:     moving robot position down  (m)
   space:    twisting robot on current position  (rad)
   PageUp:   enlarge changing unit 0.01
   PageDown: narrow down changing unit 0.01 
   Ctrl+s:    reset changing unit
  """
  self.init_pose_control_unit = {
                'up'            :(0,1,0,0.01),
                'down'          :(0,-1,0,0.01),
                'right'         :(1,0,0,0.01),
                'left'          :(-1,0,0,0.01),
                'space'         :(0,0,1,0.01),
                'PageUp'        :(0,0,0,0.01),
                'PageDown'      :(0,0,0,-0.01)}

  self.init_pose=PoseWithCovarianceStamped()
  self.init_pose.header.frame_id='map'
  self.speed=0.0
  self.angle_increase=0.0
  
#return (x,y,z,w)
 def angle_to_quat(self,angle):#in rad
  rot = PyKDL.Rotation.RotZ(angle)
  return rot.GetQuaternion()  
  
 #return RPY angle in rad
 def quat_to_angle(self,quat):
  rot = PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3])
  return rot.GetRPY()[2]
  
 def getKey(self):
  tty.setraw(sys.stdin.fileno())
  key = sys.stdin.readline(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
  return key

 def __init__(self):
  self.define()
  self.old_settings = termios.tcgetattr(sys.stdin)
  print self.terminal_msg
  data = rospy.wait_for_message('/amcl_pose',PoseWithCovarianceStamped)  
  while not rospy.is_shutdown():
   self.pose_callback(data)
  #rospy.Subscriber("turtlebot_position_in_map", Pose, self.pose_callback)
  #rospy.spin()
  

  
 def pose_callback(self,data):
  self.init_pose.pose.pose=data.pose.pose
  key = self.getKey()
  #print '\n','\n',type(key),key,'\n','\n'
  if (key == '\x1B'):
   key = sys.stdin.readline(1)
   #print key,'\n'
   if (key == '['):
    key = sys.stdin.readline(1)
    #print key,'\n'
    if 'A' in key:
     #print 'UP'
     key_value='up'
     self.init_pose.pose.pose.position.y +=self.init_pose_control_unit[key_value][1]*(self.init_pose_control_unit[key_value][3]+self.speed)
      
    if 'B' in key:
     #print 'DOWN'    
     key_value='down'
     self.init_pose.pose.pose.position.y += self.init_pose_control_unit[key_value][1]*(self.init_pose_control_unit[key_value][3]+self.speed)
      
    if 'C' in key:
     #print 'RIGHT'
     key_value='right'
     self.init_pose.pose.pose.position.x += self.init_pose_control_unit[key_value][0]*(self.init_pose_control_unit[key_value][3]+self.speed)
      
    if 'D' in key:
     #print 'LEFT'
     key_value='left'
     self.init_pose.pose.pose.position.x += self.init_pose_control_unit[key_value][0]*(self.init_pose_control_unit[key_value][3]+self.speed)   
       
      
   if (key == '5'):
    key = sys.stdin.readline(1)
    #print 'PageUp'
    key_value='PageUp'
    self.speed+=self.init_pose_control_unit[key_value][3]
    print 'speed: ',(0.01+self.speed) 
    
   if (key == '6'):
    key = sys.stdin.readline(1)
    #print 'PageDown'
    key_value='PageDown'   
    self.speed+=self.init_pose_control_unit[key_value][3]
    print 'speed: ',(0.01+self.speed) 
    
  if (key == ' '):
   #print 'space'
   key_value='space'
   self.angle_increase = self.quat_to_angle((self.init_pose.pose.pose.orientation.x,self.init_pose.pose.pose.orientation.y,self.init_pose.pose.pose.orientation.z,self.init_pose.pose.pose.orientation.w))
   #print 'current angle ', self.angle_increase
   self.angle_increase += self.init_pose_control_unit[key_value][2]*(self.init_pose_control_unit[key_value][3]+self.speed)
   
   #if self.angle_increase > numpy.pi*2:
    #self.angle_increase = 0
   
   #print 'change angle',self.angle_increase
   quat_increase = self.angle_to_quat(self.angle_increase)
   
   self.init_pose.pose.pose.orientation.x = quat_increase[0]
   self.init_pose.pose.pose.orientation.y = quat_increase[1]
   self.init_pose.pose.pose.orientation.z = quat_increase[2]
   self.init_pose.pose.pose.orientation.w = quat_increase[3]
   #else:    
    #print key
    
  if (key == '\x13'):
   print 'REST'
   self.speed=0.0
   
  if (key == '\x03'):
   #break
   rospy.signal_shutdown('Keyborad Interuption')
   
  self.init_pose.header.stamp=rospy.Time.now()  
  self.position_pub.publish(self.init_pose)
    
if __name__=='__main__':
 try:
  print "initialization system"
  init_pose_handle()
  print "process done and quit"
 except rospy.ROSInterruptException:
  rospy.loginfo("robot excel_to_coordinate node terminated.")
