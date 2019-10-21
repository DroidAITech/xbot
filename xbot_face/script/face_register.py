#!/usr/bin/env python
#coding=utf-8

import time,cv2
import os
import base64
import urllib2
import sys
import os
from Tkinter import *
import tkFileDialog
from json import *

result = {0:"成功", 1:"请求超时",2:"识别/核身解析结果错误",3:"识别解析结果错误",4:"识别解析相似度错误",
		5:"输入信息错误",6:"无该Userid对应信息",7:"获取注册人脸图片数据错误",
		8:"Base64解码错误",9:"人脸数据提取失败",10:"删除人脸失败",11:"图片过大，不超过初始配置大小（1280,720",
		12:"文件不存在",13:"人脸图片打开失败",14:"人脸已存在（注册人脸时method=normal情况下，userid重复",
		15:"未检测到网卡",16:"输入信息不合法",17:"一键开门失败",18:"文件读取失败",2019:"恢复出厂设置失败",
		2010:"数据清除失败",2021:"获取日志列表失败",22:"MAC地址不匹配"}	

class face_register():
	def __init__(self):	

		try:
			mode = sys.argv[1]
			if mode =="image":
				user_name = sys.argv[1]
				image_path = sys.argv[2]
				self.image_register(user_name)
			if mode =="camera":
				camera_index = sys.argv[2]
				camera_index = int(camera_index)
				self.camera_register(camera_index)
		except KeyboardInterrupt:
			print("	Has Exited or Finished!")
		except IndexError:	
			print("Input ERROR!")
			print("Please input the correct command and try again!")
			print("使用摄像头交互进行注册:")
			print("rosrun xbot_face 01face_register.py camera [camera_index]")
			print("使用照片进行注册:")
			print("rosrun xbot_face 01face_register.py image [user_name] ")

	# 使用照片进行注册
	def image_register(self,user_name):
		print("使用照片进行注册:")
		url = "http://192.168.8.141:8000/management/register?method=force"
		body = str()
		post_data = dict()
		post_data["Userid"] = user_name
		image_path = tkFileDialog.askopenfilename()

		with open(image_path, "rb") as fp:
			image_binary = fp.read()
			image_binary = base64.b64encode(image_binary)
			post_data["Image"] = image_binary
		if post_data:
			body = JSONEncoder().encode(post_data)
		req = urllib2.Request(url, body)
		response = urllib2.urlopen(req)
		buf = response.read()
		body = JSONDecoder().decode(buf)
		ret = body["Ret"]
		print("返回状态:"),
		print body["Ret"],
		print result[ret]
		

	# 使用摄像头交互进行注册
	def camera_register(self,camera_index):		
		print("使用摄像头交互进行注册:")
		url = "http://192.168.8.141:8000/management/register?method=normal"
		cap = cv2.VideoCapture(camera_index)
		body = str()
		post_data = dict()
		while(True):
			
			ret, frame = cap.read()

			
			cv2.imshow('frame',frame)
			if cv2.waitKey(1) & 0xFF == ord('r'):
				cv2.imwrite('tmp.jpg',frame)
				tip = '请用拼音输入你的名字:\n'
				name = input(tip)
				post_data["Userid"] = name
				with open('tmp.jpg', "rb") as fp:
					image_binary = fp.read()
					image_binary = base64.b64encode(image_binary)
					post_data["Image"] = image_binary
				if post_data:
						body = JSONEncoder().encode(post_data)
				req = urllib2.Request(url, body)
				response = urllib2.urlopen(req)
				buf = response.read()
				body = JSONDecoder().decode(buf)
				ret = body["Ret"]
				print("返回状态:"),
				print body["Ret"],
				print result[ret]
			elif cv2.waitKey(1) & 0xFF == ord('q'):
				break

		# When everything done, release the capture
		cap.release()
		cv2.destroyAllWindows()


if __name__ == '__main__':
	face_register()
	
