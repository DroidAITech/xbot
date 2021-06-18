#!/usr/bin/env python
#coding=utf-8

import os
import rospy
from std_msgs.msg import Bool, UInt8, String
from xbot_talker.msg import recog_result
from xbot_talker.srv import CallChat
from geometry_msgs.msg import Twist

import sys
#reload(sys)
#sys.setdefaultencoding('utf-8')


class VoiceControl():
    def __init__(self):
        rospy.init_node('voice_control',anonymous=False)
        self.call_chat_srv =  rospy.ServiceProxy('/talker/chat',CallChat)
        self.speech_recog_sub = rospy.Subscriber('/talker/offline_recog_result', recog_result, self.speechCB)
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

        ## 调用chat服务
        res = self.call_chat_srv(True)
        if res:
            print "success to calll service /talker/chat"
        else:
            print "failed to calll service /talker/chat"
        rospy.spin()

    def speechCB(self, msg):
        print msg.recog_result
        print msg.recog_accuracy
        if msg.recog_accuracy > 30:
            if msg.recog_result == '向左旋转' or msg.recog_result == '向左转' :
                print "turn left"
                cmd_twist = Twist()
                cmd_twist.angular.z = 0.5
                self.twist_pub.publish(cmd_twist)
            if msg.recog_result == '向前走一步' or msg.recog_result == '向前走':
                print "move forward"
                cmd_twist = Twist()
                cmd_twist.linear.x  = 0.2
                self.twist_pub.publish(cmd_twist)
            ## 完善其他处理代码

          ## 添加一轮对话结束后向/talker/enable_asr发布True消息开启新一轮对话代码

if __name__ == '__main__':
    VoiceControl()
