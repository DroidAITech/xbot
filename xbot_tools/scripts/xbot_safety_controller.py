#!/usr/bin/env python
#coding=utf-8
"""
xbot超声近场测距安全距离控制程序

"""

import rospy,std_msgs.msg
from geometry_msgs.msg import Twist
from xbot_msgs.msg import DockInfraRed
from move_base_msgs.msg import MoveBaseAction

class xbot_safety_controller():
  """docstring for xbot_safety_controller"""
  def __init__(self):
    rospy.init_node('xbot_safety_controller')
    self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.pub = rospy.Publisher('/cmd_vel_mux/input/safety_controller', Twist, queue_size = 1)
    rospy.Subscriber("/sensor/echo_data", DockInfraRed, self.echo_dataCB)
    rospy.spin()



  def echo_dataCB(self,echo_data):
    # print echo_data.center_near
    if ehco_data.danger:
      cmd = Twist()
      cmd.linear.x = 0
      cmd.angular.z = 0
      self.pub.publish(cmd)
      # self.move_base.cancel_goal()




if __name__=='__main__':
 try:
  rospy.loginfo ("initialization system")
  xbot_safety_controller()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
