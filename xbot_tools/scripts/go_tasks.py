#!/usr/bin/env python
#coding=utf-8
"""
oneway instance tasks

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy
import getpass
import actions_reference
import actionlib
import subprocess
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatus,GoalStatusArray
from rosgraph_msgs.msg import Log
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped,Pose

class cruise_modle():
 def define(self):
  self.period=rospy.Duration(1)
  self.state_define={
  GoalStatus.ACTIVE:      'ACTIVE: TASK goal is currently being processed by the server, PLEASE WAIT for SUCCEEDED',
  GoalStatus.SUCCEEDED:   'SUCCEEDED: TASK goal was achieved successfully by the server, PLEASE SEND A NEW GOAL'
  }


  self.error_state_define={
  GoalStatus.PENDING:     'PENDING: TASK goal has yet to be processed by the server, PLEASE WAIT',
  GoalStatus.PREEMPTED:   'PREEMPTED: TASK goal received a cancel request after it started executing and has since completed its execution, PLEASE SEND A NEW GOAL',
  GoalStatus.ABORTED:     'ABORTED: TASK goal was aborted during execution by the server due to some failure, PLEASE SEND A NEW GOAL',
  GoalStatus.REJECTED:    'REJECTED: TASK goal was rejected by the server without being processed, because the goal was unattainable or invalid, PLEASE SEND A NEW GOAL',
  GoalStatus.PREEMPTING:  'PREEMPTING: TASK goal received a cancel request after it started executing and has not yet completed execution, PLEASE SEND A NEW GOAL',
  GoalStatus.RECALLING:   'RECALLING: TASK goal received a cancel request before it started executing, but the server has not yet confirmed that the goal is canceled, PLEASE SEND A NEW GOAL',
  GoalStatus.RECALLED:    'RECALLED: TASK goal received a cancel request before it started executing and was successfully cancelled, PLEASE SEND A NEW GOAL',
  GoalStatus.LOST:        'LOST: TASK is determineed that a goal is LOST. This should not be sent over the wire by an server, PLEASE SEND A NEW GOAL'
  }


  self.last_position=Point()
  self.current_position=Point()
  self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  self.move_base.wait_for_server()
  self.log_info=Log()
  self.current_odom=PoseWithCovarianceStamped()
  
 def odom_callback(self,odom):
  self.current_odom.pose.pose=odom
 
 def marker_callback(self,marker_point): 
  self.marker_point=marker_point
  self.move_base.cancel_goal()
  actions_reference.go_single_marker(self.current_odom,self.marker_point)
  
 def status_callback(self,goal_state):
  state_list=goal_state.status_list
  if state_list != []:
   state=state_list[0]
   self.state=state
   if state.status==GoalStatus.ACTIVE or state.status==GoalStatus.SUCCEEDED:
    if self.state_define[state.status] not in self.log_info.msg:
     rospy.loginfo(self.state_define[state.status])
     self.move_base.cancel_goal()
     pass

   if state.status!=GoalStatus.ACTIVE and state.status!=GoalStatus.SUCCEEDED:
    if self.error_state_define[state.status] not in self.log_info.msg:
     rospy.loginfo(self.error_state_define[state.status])
     self.move_base.cancel_goal()
     pass




 def Log_callback(self,log_info):
  self.log_info=log_info
 
 def timer(self,event):
  (x_state,y_state,z_state)=actions_reference.position_compara(self.last_position,self.current_position)
  if (x_state,y_state,z_state)!=(True,True,True):
   self.last_position=self.current_position
  else:
   self.move_base.cancel_goal()
   pass



 def __init__(self):
  rospy.init_node('go_tasks')
  self.marker_point=Marker()
  self.define()
  rospy.Timer(self.period, self.timer)
  
  rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)
  #rospy.Subscriber("odom", Odometry, self.odom_callback)
  rospy.Subscriber("xbot_position_in_map", Pose, self.odom_callback)
  rospy.Subscriber("ui_marker", Marker,self.marker_callback)
  rospy.spin()

if __name__ == '__main__':
 try:
  rospy.loginfo( "initialization system")
  cruise_modle()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("follower node terminated.")

