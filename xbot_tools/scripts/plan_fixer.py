#!/usr/bin/env python
#coding=utf-8

"""
路径规划补偿（move base dijkstar 用）

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

import rospy
import numpy
import Queue
import copy
from threading import Lock
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped

class ClearParams():
    def __init__(self):
        rospy.loginfo('Cleaning Parameters...')
        rospy.delete_param('~PlanTopic')
        rospy.delete_param('~OdomTopic')
        rospy.delete_param('~PositionFree')
        rospy.delete_param('~ActionPlanTopic')
        rospy.delete_param('~GoalTopic')


class PlanFixer():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.SubTopic, Path, self.RawPathCB)
        rospy.Subscriber('%s' % self.OdomTopic, Pose, self.OdomCB)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~PlanTopic'):
            rospy.set_param('~PlanTopic', '/move_base/NavfnROS/plan')
        if not rospy.has_param('~OdomTopic'):
            rospy.set_param('~OdomTopic', '/robot_position_in_map')
        if not rospy.has_param('~PositionFree'):
         rospy.set_param('~PositionFree', 0.5)
        if not rospy.has_param('~ActionPlanTopic'):
            rospy.set_param('~ActionPlanTopic', '/move_base/action_plan')
        if not rospy.has_param('~GoalTopic'):
            rospy.set_param('~GoalTopic', '/clicked_point')

        self.GoalTopic = rospy.get_param('~GoalTopic')
        self.ActionPlanTopic = rospy.get_param('~ActionPlanTopic')
        self.SubTopic = rospy.get_param('~PlanTopic')
        self.OdomTopic = rospy.get_param('~OdomTopic')
        self.PositionFree = rospy.get_param('~PositionFree')

        self.locker = Lock()
        self.Fixer_Thread = False
        self.Store_Goal = Queue.Queue(maxsize=1)
        self.seq = None

    def OdomCB(self, data):
        with self.locker:
            self.odom = data.position

    def RawPathCB(self, data):
        with self.locker:
            try:
                self.PathHandle(data)
            except:
                rospy.loginfo('waiting for odom data...')

    def PathHandle(self, data):
        self.seq = data.header.seq
        path = data.poses
        if self.Fixer_Thread:
            if len(path) <= 3:
                if not self.Store_Goal.empty():
                    goal = self.Store_Goal.get()
                    self.RestoreGoal(goal)
                    self.Fixer_Thread = False
                else:
                    rospy.loginfo('没有在储目标')
                    self.PubFixPlan(data)
            else:
                rospy.loginfo('发布修正路径')
                self.PubFixPlan(data)

        else:
            origin = path[0].pose.position
            # 判断是否起始点为路径规划起始点
            # RawOdom = rospy.wait_for_message(self.OdomTopic, Pose)
            OdomData = self.odom
            if self.OriginCheck(origin, OdomData):
                if len(data.poses) > 3:
                    #print len(data.poses)
                    rospy.loginfo('发布原始路径')
                    self.PubFixPlan(data)
            else:
                rospy.loginfo('执行fixer中...')
                self.Fixer(data)


    def RestoreGoal(self, data):
        rospy.loginfo('重载目标...')
        goal = PointStamped()
        goal.point = data
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        self.seq += 1
        goal.header.seq = self.seq
        self.PubGoal(goal)

    def PubFixPlan(self, data):
        PubPlan = rospy.Publisher(self.ActionPlanTopic, Path, queue_size=1)
        PubPlan.publish(data)

    def Fixer(self, data):
        self.Fixer_Thread = True
        rospy.loginfo('存储目标中..')
        self.Store_Goal.put(copy.deepcopy(data.poses[-1].pose.position))
        NewGoal = PointStamped()
        NewGoal.point = data.poses[1].pose.position
        NewGoal.header.frame_id = 'map'
        self.seq += 1
        NewGoal.header.seq = self.seq
        rospy.loginfo('产生新目标，制定新路径...')
        self.PubGoal(NewGoal)

    def PubGoal(self, goal):
        PubPlan = rospy.Publisher(self.GoalTopic, PointStamped, queue_size=1)
        rospy.sleep(0.5)
        PubPlan.publish(goal)

    def OriginCheck(self, origin, current_position):
        x_drift = round((origin.x - current_position.x), 3)
        y_drift = round((origin.y - current_position.y), 3)
        total_drift = round(numpy.sqrt(x_drift ** 2 + y_drift ** 2), 3)
        if total_drift > self.PositionFree:
            return False
        else:
            return True

if __name__=='__main__':
    rospy.init_node('Planfixer')
    try:
        rospy.loginfo( "initialization system")
        PlanFixer()
        ClearParams()
        rospy.loginfo ("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
