#!/usr/bin/env python
#coding=utf-8
"""
发布鼠标点击位置为运动目标

"""

import rospy,std_msgs.msg
from geometry_msgs.msg import PointStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal

class clicked_goal():
  """docstring for clicked_goal"""
  def __init__(self):
    rospy.init_node('clicked_goal')
    # self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)
    rospy.Subscriber("/clicked_point", PointStamped, self.clicked_pointCB)
    rospy.spin()



  def clicked_pointCB(self,clicked_point):
    goal=PoseStamped()
    goal.header=clicked_point.header
    goal.pose.position=clicked_point.point
    goal.pose.orientation.x=0
    goal.pose.orientation.y=0
    goal.pose.orientation.z=0
    goal.pose.orientation.w=0.001
    self.pub.publish(goal)




if __name__=='__main__':
 try:
  rospy.loginfo ("initialization system")
  clicked_goal()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
