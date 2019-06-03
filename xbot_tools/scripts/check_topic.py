#!/usr/bin/env python
import rospy
import geometry_msgs.msg


def topicCB(msg):
	print len(msg.poses)



if __name__ == '__main__':
		rospy.init_node('check_topic')
		global img_pub
		rospy.Subscriber('/particlecloud', geometry_msgs.msg.PoseArray, topicCB)
		rospy.spin()