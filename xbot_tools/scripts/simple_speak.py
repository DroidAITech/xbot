#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot."""


from pyaudio import PyAudio, paInt16
import json,os,sys,requests,vlc,rospy,getpass
import numpy as np 
from std_msgs.msg import String
#from pydub import AudioSegment

"sudo apt-get install python-pygame"

class speeker():
 def __init__(self):
  self.define()
  while not rospy.is_shutdown():
   data = rospy.wait_for_message('speak_string', String)
   speak_string = data.data
   self.speek(speak_string)  
  
 def speek(self,speak_string):
  #获取token
  requestData = {       "grant_type":           self.Grant_type, 
                        "client_id":            self.Api_Key, 
                        "client_secret":        self.Secrect_Key}
  
  result = requests.post(url = self.Token_url, data = requestData)
  
  token_data = json.loads(result.text)
  
  #self.Print_Response(token_data)
  
  if 'access_token' in token_data:  
   token = token_data['access_token']    #获取的token
   rospy.loginfo('token获取成功\n')#, 'token: ', token , '\n')
  else:
   rospy.loginfo("token获取失败\n")
  #tex = self.get_text() #for testing
  #tex = "魔兽世界" #for testing
  tex=speak_string

  #语音合成
  SpkData = {           "tex":          tex, 
                        "lan":          self.LAN, 
                        "tok":          token,
                        "ctp":          self.CTP,
                        "cuid":         self.USER_ID,
                        "spd":          self.SPEED,
                        "pit":          self.PIT,
                        "vol":          self.VOL,
                        "per":          self.PER['women']}

  re_voice=[]

  re = requests.post(url = self.Speeker_url, data = SpkData)

  #print 'header'
  #self.Print_Response(re.headers)
  #print 'header\n'
  
  if  'audio/mp3' in re.headers['content-type']: 
    
   file_=self.write_mp3(re)

  elif 'application/json' in  re.headers['content-type']: 
 
   spker_data=json.loads(re.text)
  
   self.Print_Response(spker_data)
  
   print '\n合成出错，原因： ',self.error_reason[spker_data[u'err_no']], '\n'
    
    
  if os.path.exists(r'%s'%file_):
   self.play_video(file_)
  
  
 def define(self):
  self.error_reason = { 500:    '不支持输入',
                        501:    '输入参数不正确',
                        502:    'token验证失败',
                        503:    '合成后端错误'}
                     
  self.PER = {'women': 0, 'man': 1} #发音人选择，取值0-1, 0为女声，1为男声，默认为女声
                        
  if rospy.has_param('~CTP'):
   pass
  else:
   rospy.set_param('~CTP', 1)

  if rospy.has_param('~LAN'):
   pass
  else:
   rospy.set_param('~LAN', 'zh')
                 
  if rospy.has_param('~USER_ID'):
   pass
  else:
   rospy.set_param('~USER_ID', '8168466')                     
                        
  if rospy.has_param('~SPEED'):
   pass
  else:
   rospy.set_param('~SPEED', 5)    
   
  if rospy.has_param('~PIT'):
   pass
  else:
   rospy.set_param('~PIT', 5)  
   
  if rospy.has_param('~VOL'):
   pass
  else:
   rospy.set_param('~VOL', 5)                     

  if rospy.has_param('~Api_Key'):
   pass
  else:
   rospy.set_param('~Api_Key', "pmUzrWcsA3Ce7RB5rSqsvQt2")    
 
  if rospy.has_param('~Secrect_Key'):
   pass
  else:
   rospy.set_param('~Secrect_Key', "d39ec848d016a8474c7c25e308b310c3")

  if rospy.has_param('~Grant_type'):
   pass
  else:
   rospy.set_param('~Grant_type', "client_credentials")

  if rospy.has_param('~Token_url'):
   pass
  else:
   rospy.set_param('~Token_url', "https://openapi.baidu.com/oauth/2.0/token")
   
  if rospy.has_param('~Speeker_url'):
   pass
  else:
   rospy.set_param('~Speeker_url', "http://tsn.baidu.com/text2audio")

  if rospy.has_param('~FORMAT'):
   pass
  else:
   rospy.set_param('~FORMAT', "mp3")
  
  self.CTP = rospy.get_param('~CTP') # default 1
  
  self.LAN = rospy.get_param('~LAN') # default 'zh'
  
  self.USER_ID = rospy.get_param('~USER_ID') # default '8168466'             

  self.SPEED = rospy.get_param('~SPEED') # default 5 语速，取值0-9，默认为5中语速

  self.PIT = rospy.get_param('~PIT')     # default 5 音调，取值0-9，默认为5中语调

  self.VOL = rospy.get_param('~VOL')     # default 5 音量，取值0-9，默认为5中音量

  self.Api_Key = rospy.get_param('~Api_Key') # default "pmUzrWcsA3Ce7RB5rSqsvQt2"
  
  self.Secrect_Key = rospy.get_param('~Secrect_Key') # default "d39ec848d016a8474c7c25e308b310c3"
  
  self.Grant_type = rospy.get_param('~Grant_type') # default "client_credentials"
  
  self.Token_url = rospy.get_param('~Token_url') # default 'https://openapi.baidu.com/oauth/2.0/token'
  
  self.Speeker_url = rospy.get_param('~Speeker_url') # default 'http://tsn.baidu.com/text2audio'
 
  self.FORMAT = rospy.get_param('~FORMAT') # default 'mp3'
  
  #self.SAMPLING_RATE =       16000    #取样频率
  
  #self.pub = rospy.Publisher('speak_status', String, queue_size=10)
  
 def Print_Response(self, data):
  for i in data:
   print '\t',i, ': ', data[i]
  
 def write_mp3(self,data):
  #path=self.current_path()
  #path='/home/turtlebot/xu_slam/src/simple_voice/src/'#testing
  count=getpass.getuser()
  path='/home/%s/cafe_robot_single/src/simple_voice/src/'%count
  #print type(data.content),len(data.content)
  
  Voice_String=data.content
  
  file_=self.savemp3('%s'%path+'testing1',Voice_String)
  
  return file_
  
 def current_path(self):
 
  path=sys.path[0] 
  
  if os.path.isdir(path):
   return path
   
  elif os.path.isfile(path):
   return os.path.dirname(path)

 def get_text(self):
  return raw_input('请输入想要合成的话语： ')
  
 def savemp3(self,filename,Voice_String):
  wf = open(filename+'.%s'%self.FORMAT, 'w') 
  wf.write(Voice_String) 
  wf.close() 
  return filename+'.%s'%self.FORMAT


 def play_video(self,file_):
  #print '\n start speaking ', "file://%s"%file_
  rospy.loginfo('start speaking ')
  player = vlc.MediaPlayer("file://%s"%file_)
  player.play()   
  rospy.sleep(1)
  while player.is_playing():
   pass
   #self.pub.publish('PENDING')
  rospy.loginfo('done\n')
  
if __name__=="__main__":
 rospy.init_node('simple_speaker')
 rospy.loginfo( "initialization system")
 speeker()
 rospy.loginfo( "process done and quit")
