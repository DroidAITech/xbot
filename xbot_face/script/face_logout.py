#!/usr/bin/env python
#coding=utf-8

import time,cv2
import os
import base64
import urllib2
import sys
from json import *

result = {0:"成功", 1:"请求超时",2:"识别/核身解析结果错误",3:"识别解析结果错误",4:"识别解析相似度错误",
		5:"输入信息错误",6:"无该Userid对应信息",7:"获取注册人脸图片数据错误",
		8:"Base64解码错误",9:"人脸数据提取失败",10:"删除人脸失败",11:"图片过大，不超过初始配置大小（1280,720",
		12:"文件不存在",13:"人脸图片打开失败",14:"人脸已存在（注册人脸时method=normal情况下，userid重复",
		15:"未检测到网卡",16:"输入信息不合法",17:"一键开门失败",18:"文件读取失败",2019:"恢复出厂设置失败",
		2010:"数据清除失败",2021:"获取日志列表失败",22:"MAC地址不匹配"}	

class face_logout():
	def __init__(self):	
		try:
			user_name= sys.argv[1]
			if user_name =="all":
				self.logoutall()
			else:
				self.logoutuser(user_name)

		except KeyboardInterrupt:
			print("	Has Exited or Finished!")
		except IndexError:
			print("Please input the correct command and try again!")

	def logoutall(self):
		url = "http://192.168.8.141:8000/management/logoutall"
		req = urllib2.Request(url)
		response = urllib2.urlopen(req)
		buf = response.read()
		body = JSONDecoder().decode(buf)
		print body

	def logoutuser(self,user_name):
		userid = user_name
		url="http://192.168.8.141:8000/management/logout?userid="+userid
		req = urllib2.Request(url)
		response = urllib2.urlopen(req)
		buf = response.read()
		body = JSONDecoder().decode(buf)
		ret = body["Ret"]
		print("返回状态:"),
		print body["Ret"],
		print result[ret]



if __name__ == '__main__':
	face_logout()





