#!/usr/bin/env python
#coding=utf-8
"""
this program is used to launch all navigation staff

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy
import os
import subprocess
from rosgraph_msgs.msg import Log

class Xbot_cruise():
 def define(self):
 
  self.Amcl_ready = False
  self.Control_ready = False
  self.Mobile_ready = False
  self.All_Done = False
  self.Path_ready = False
  
  self.MobileBaseRequiredTopics = ['/rosout', '/cmd_vel_mux/parameter_descriptions', '/cmd_vel_mux/parameter_updates', '/cmd_vel', '/cmd_vel_mux/active', '/mobile_base_nodelet_manager/bond', '/tf', '/joint_states', '/mobile_base/controller_info', '/mobile_base/events/power_system', '/mobile_base/events/robot_state', '/mobile_base/sensors/core', '/mobile_base/sensors/dock_ir', '/mobile_base/sensors/imu_data', '/mobile_base/sensors/imu_data_raw', '/mobile_base/debug/raw_data_command', '/mobile_base/debug/raw_data_stream', '/mobile_base/debug/raw_control_command', '/odom']
  self.GetPath = {'file' : 'path.py', 'msg' : 'path getted', 'name' : '/path_generator'}
  
  
 ############## LAUNCH ##################
 def RobotBringup(self):
  subprocess.Popen('roslaunch xbot_bringup xbot_minimal.launch',shell=True)

 def get_path(self):
  subprocess.call('rosrun machine path.py',shell=True)

 def AmclLaunch(self):
  subprocess.Popen('roslaunch machine Xbot_amcl.launch',shell=True)

 def robot_controller(self):
  subprocess.Popen('roslaunch machine robot_controller_cruise.launch',shell=True)
 
 def RVIZ(self):
  subprocess.Popen('roslaunch machine 3D_RVIZ.launch',shell=True)
  
 #################### checher #####################
 def mobile_ready(self, data):
  if set(self.MobileBaseRequiredTopics).intersection(data.topics) == set(self.MobileBaseRequiredTopics):
   return True
  else:
   return False
 
 def path_ready(self, data):
  if data.file == self.GetPath['file'] and data.name == self.GetPath['name'] and data.msg = self.GetPath['msg']:
   return True
  else:
   return False
     
 def if_amcl_ready(self):
  if 'odom received' in self.rosout_msg:
   self.amcl_ready=True

 def if_uimarker_ready(self):
  if '请使用publish point选出想要标记的地方' in self.rosout_msg:
   self.uimarker_ready=True
     
     
     
 ###################################
 #            All Done             #
 #               |no               #
 #        mobile base launch       #
 #               |yes              #
 #          get map path           #
 #               |yes              #
 #           amcl launch           #
 #               |yes              #
 #         controller launch       #
 #               |yes              #
 #            All Done             #
 ###################################
 
 def LOG_CB(self, data):
  if not self.All_Done:
   #testing if mobile base launched  
   if self.Mobile_ready:
    self.Mobile_ready = self.mobile_ready(data)
    self.get_path()
    self.Path_ready = self.path_ready(data)
   if self.Path_ready:
    self.AmclLaunch()
     if 
    
   else:
    rospy.sleep(2)
    if not self.mobile_ready(data):
     self.RobotBringup()
    else:
     pass

   
  if self.getting_path(data):
   self.AmclLaunch()

  self.if_amcl_ready()
  if self.amcl_ready:
   self.amcl_ready=False
   
  self.if_uimarker_ready()
  if self.uimarker_ready:
   self.uimarker_ready=False
   self.robot_controller()
   rospy.sleep(2)
   self.finish=True
   
  if self.finish:
   self.finish=False

   self.RVIZ()

  
 def __init__(self):
  self.define()
  rospy.Subscriber('/rosout',Log, self.LOG_CB)
  rospy.spin()

if __name__=='__main__':
 rospy.init_node('Xbot_cruise')
 try:
  rospy.loginfo ("initialization system")
  cafe_robot_total()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
