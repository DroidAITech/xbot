#!/usr/bin/env python
from cv_bridge import CvBridge
import std_msgs.msg
import nav_msgs.msg
import numpy as np
import rospy
import cv2
import cv2
import base64

map_img = None
img_pub = None
bridge = None
robot_pose = None
def map_to_arr(map_msg):
	map_arr = np.random.random((map_msg.info.height, map_msg.info.width,3))
	arr = np.reshape(map_msg.data, (map_msg.info.height, map_msg.info.width))

	# -1 is unknown. If we cast that to a uint8, it won't look right.
	# map_saver uses 205 for unknown values, so let's change all -1 cells to 205
	#unknown space
	map_arr[:,:,0][arr == -1] = 100
	map_arr[:,:,1][arr == -1] = 100
	map_arr[:,:,2][arr == -1] = 100

	#obstacles
	map_arr[:,:,0][arr == 100] = 0
	map_arr[:,:,1][arr == 100] = 0
	map_arr[:,:,2][arr == 100] = 0

	#known space
	map_arr[:,:,0][arr == 0] = 121
	map_arr[:,:,1][arr == 0] = 213
	map_arr[:,:,2][arr == 0] = 117
	add_robot(robot_pose,map_arr)
	img =  np.uint8(np.flipud(map_arr))
	
	img_encode = cv2.imencode('.png', img)[1]
	data_encode = np.array(img_encode)
	# str_encode = data_encode.tostring()
	str_encode = base64.b64encode(data_encode)
	return str_encode

def map_cb(msg):
	global map_img
	map_img = msg
	# img_pub.publish(map_img)
	
def robopose_cb(msg):
	global robot_pose
	robot_pose = msg.pose.pose

def add_robot(pose, arr):
	y=int(round(pose.position.x/0.05)+498)
	x=int(round(pose.position.y/0.05)+498)
	print x,y
	arr[x-3:x+3,y-3:y+3,0]=0
	arr[x-3:x+3,y-3:y+3,1]=0
	arr[x-3:x+3,y-3:y+3,2]=255
	


if __name__ == '__main__':
	rospy.init_node('map2img')
	# img=cv2.imread('map.png')
	# print base64.b64encode(img)
	# bridge = CvBridge()
	global img_pub
	img_pub = rospy.Publisher('/base64_img/map_img', std_msgs.msg.String,queue_size=1)
	rospy.Subscriber('odom', nav_msgs.msg.Odometry, robopose_cb)
	rospy.Subscriber('map', nav_msgs.msg.OccupancyGrid, map_cb)
	# rospy.spin()
	rate = rospy.Rate(10)
	rospy.loginfo('Waiting for map')    
	while map_img is None:
		rate.sleep()
	rospy.loginfo('Got map')        
	while not rospy.is_shutdown():
		map = map_to_arr(map_img)
		img_pub.publish(map)
	#     # cv2.imshow('img',map_img)
	#     # cv2.waitKey(200)
		rate.sleep()