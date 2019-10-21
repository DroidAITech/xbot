#!/usr/bin/env python
#coding=utf-8
import numpy as np
import cv2, os, time, base64, urllib2, rospy
from json import *
# ros msg
from xbot_face.msg import FaceResult
from std_msgs.msg import String, UInt32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#人脸识别盒子的IP地址以及接口,详见doc中的天眼魔盒API文档
url = "http://192.168.8.141:8000/recognition"



class face_recog():
	"""docstring for face_recog"""
	def __init__(self):
#       声明节点订阅与发布的消息
		self.face_result_pub = rospy.Publisher('/xbot/face_result',FaceResult,queue_size=1)
		rospy.Subscriber('/xbot/camera/image', Image, self.imageCB)
		self.bridge = CvBridge()
		rospy.spin()




	def imageCB(self, msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)

		result_msg = FaceResult()
		cv2.imwrite('tmp.jpg',cv_image)
		with open("tmp.jpg", "rb") as fp:
			image_binary = fp.read()
			image_binary = base64.b64encode(image_binary)
			post_data = {"Image":image_binary}
			body = JSONEncoder().encode(post_data)
			req = urllib2.Request(url, body)
			response = urllib2.urlopen(req).read()
			body = JSONDecoder().decode(response)
			# rospy.loginfo(body)
#                如果没有检测到人脸,人脸次数重置,
		# rospy.logwarn(body)
		if body['Id'] == 'UNKNOWN' or body['Id'] == 'None':#no people in frame
			result_msg.face_exist = 0
			result_msg.name = body['Id']
			result_msg.confidence = 0
			self.face_result_pub.publish(result_msg)

#                检测到已注册人脸,人脸次数+1,继续,连续累积次数(中间不出现重置)大于10则确认该识别结果
		else:#registered people in frame
			result_msg.face_exist = 1
			result_msg.name = body['Id']
			result_msg.confidence = body['Confidence']
			self.face_result_pub.publish(result_msg)
			# detected face, but not registered face
		# else:
		# 	result_msg.face_exist = 1
		# 	result_msg.name = 'unknown'
		# 	result_msg.confidence = body['Confidence']
		# 	self.face_result_pub.publish(result_msg)




if __name__ == '__main__':
	rospy.init_node('face_recog')
	try:
		rospy.loginfo('face recogition initialized...')
		face_recog()
	except rospy.ROSInterruptException:
		rospy.loginfo('face recogition initialize failed, please retry...')

