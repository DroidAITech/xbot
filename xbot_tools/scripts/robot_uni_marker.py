#!/usr/bin/env python
#coding=utf-8
""" 
this code is used for making one marker in map

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped,Pose


class marker():
 def define(self):
  self.marker=Marker()
  self.marker.color.r=1.0
  self.marker.color.g=0.0
  self.marker.color.b=0.0
  self.marker.color.a=1.0
  self.marker.id = 0
  self.marker.ns='robot_uni_marker'
  self.marker.scale.x=0.1
  self.marker.scale.y=0.1
  self.marker.scale.z=0.1
  self.marker.header.frame_id='map'
  self.marker.type=Marker.TEXT_VIEW_FACING
  self.marker.lifetime = rospy.Duration(0)
  self.marker_pub=rospy.Publisher("robot_uni_marker",Marker,queue_size=1)

 def pose_callback(self, data):
  self.marker.header.stamp =rospy.Time.now()
  #self.marker.pose=data.pose.pose
  self.marker.pose=data
  self.marker.pose.position.z=0.5
  if rospy.has_param('~robot_laber'):
   self.marker.text=rospy.get_param('~robot_laber')
  else:
   rospy.set_param('~robot_laber','default_robot')
   self.marker.text=rospy.get_param('~robot_laber')
  self.marker_pub.publish(self.marker)
   
 def __init__(self):
  rospy.init_node('robot_uni_marker')
  self.define()
  rospy.Subscriber("xbot_position_in_map", Pose, self.pose_callback)
  rospy.spin()


if __name__=='__main__':
 try:
  rospy.loginfo ("initialization system")
  marker()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
