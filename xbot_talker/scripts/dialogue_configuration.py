#!/usr/bin/env python
#coding=utf-8

import json
import os
import codecs
import rospy
import subprocess
from std_msgs.msg import Bool, UInt8, String
from xbot_talker.msg import actionmode, actionmode_ros, action, action_ros, dialogue_ros, dialogue
from xbot_talker.srv import *
import chardet
import datetime
import csv

import sys
reload(sys)     
sys.setdefaultencoding('utf-8')


class DialogueConfig():
    def __init__(self):
        rospy.init_node('dialogue_config',anonymous=False)
        self.get_actionmode_srv = rospy.Service('/app/get_actionmode',call_actionmode,self.call_actionmodes)
        self.get_action_srv = rospy.Service('/app/get_action',call_action,self.call_actions)
        self.add_dialogue_srv = rospy.Service('/app/add_dialogue',add_dialogue,self.add_dialogues)
        self.get_history_srv = rospy.Service('/app/history_record',history_record,self.get_history)
        self.roolback_srv = rospy.Service('/app/roolback_dialogue',roolback_dialogue,self.roolback_dialogue)

        rospy.spin()


    # 配置nlp对话库csv文件并生效
    def nlp_config(self, keyword, answer,action,action_mode):
        rospy.wait_for_service('/xbot_talker/nlp_dialog_config')
        nlp_dialog_config = rospy.ServiceProxy('/xbot_talker/nlp_dialog_config', nlpdialog_config)
        res=nlp_dialog_config(keyword,answer,action,action_mode)
        return res.error_code


    # 配置asr重置语法参数
    def asr_config(self,keyword):
        rospy.wait_for_service('/xbot_talker/asr_keyword_config')        
        asr_keyword_config = rospy.ServiceProxy('/xbot_talker/asr_keyword_config', keyword_config)
        res=asr_keyword_config(keyword)
        return res.error_code       


    # 获取动作模式字典信息
    def call_actionmodes(self, req):
        filename = os.popen('rospack find xbot_talker').read().strip() + '/defaultconfig/json/action_mode.json'
        if req.get_actionmode_req:
            with open(filename, 'r') as f:
                data = json.load(f)
            actionmodes = data['ActionMode']
            actionmode_list = actionmode_ros()
            for i in range(len(actionmodes)):
                tem = actionmode()
                tem.actionmode_desc = actionmodes[i]['description']
                tem.actionmode_name = actionmodes[i]['name']
                tem.actionmode_flag = actionmodes[i]['flag']
                actionmode_list.actionmode_ros.append(tem)
            print(actionmode_list)
            return call_actionmodeResponse(actionmode_list)
        else:
            rospy.logwarn("error: If you want to get the actionmodes, please set the get_actionmode_req True.")

    # 获取动作字典信息
    def call_actions(self, req):
        filename = os.popen('rospack find xbot_talker').read().strip() + '/defaultconfig/json/action.json'
        if req.get_action_req:
            with open(filename, 'r') as f:
                data = json.load(f)
            actions = data['Action']
            action_list = action_ros()
            for i in range(len(actions)):
                tem = action()
                tem.action_desc = actions[i]['description']
                tem.action_name = actions[i]['name']
                tem.action_flag = actions[i]['flag']
                action_list.action_ros.append(tem)
            rospy.loginfo(action_list)
            return call_actionResponse(action_list)
        else:
            rospy.logwarn("error: If you want to get the actions, please set the get_action_req True.")

    # 保存对话配置并生效
    def add_dialogues(self, req):
        filename = os.popen('rospack find xbot_talker').read().strip() + '/defaultconfig/json/dialogue.json'
        with open(filename, 'r') as f:
            data = json.load(f)
        dialogues = data['Dialogue']
        print(req)
        # 判断keyword是否已存在
        for i in range(len(dialogues)):
            if req.keyword == dialogues[i]['keyword']:
                rospy.logwarn("False: This keyword already exist!")
                return add_dialogueResponse(1)
        # 获取时间信息
        now_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # 添加对话配置到json文件
        new_dialogue = {'keyword':req.keyword,'answer':req.answer,'action':req.action,'action_mode':req.action_mode,'update_time':now_time}
        dialogues.append(new_dialogue)
        data['Dialogue'] = dialogues
        with codecs.open(filename, 'w', 'utf-8') as file:  
            json.dump(data, file, indent=4, ensure_ascii=False)
        # 配置keyword到bnf文件
        print(chardet.detect(req.keyword))
        filename_bnf = os.popen('rospack find xbot_talker').read().strip() + '/defaultconfig/grammar.bnf'
        with open(filename_bnf,'r') as f:
            data_bnf = f.read()
        print(chardet.detect(data_bnf))
        content = data_bnf.rfind(';')
        if content != -1:
            data_bnf = data_bnf[:content]+'|'+req.keyword+data_bnf[content:]
        with open(filename_bnf,'w') as f:
            f.write(data_bnf)
        add_dialogue_enable = True
        rospy.loginfo("Dialogue config【" + req.keyword + "】successful!")
        # 启动build_grammar.launch使新加keyword生效
        home_dir = "/home"
        build_grammar_command = 'roslaunch xbot_talker build_grammar.launch'
        build_grammar = subprocess.Popen(build_grammar_command, shell=True,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            cwd=home_dir)
        rospy.sleep(1)

        errcorde = self.nlp_config(req.keyword,req.answer,req.action,req.action_mode)
        if errcorde != 0:
            rospy.logwarn("False: This keyword already exist in answer_dic!")
            return add_dialogueResponse(1)

        errcorde = self.asr_config(req.keyword)
        if errcorde != 0:
            rospy.logwarn("False: reset keyword failed!")
            return add_dialogueResponse(2)

        return add_dialogueResponse(0)


    # 获取对话配置最近10条历史记录功能，查询并返回所有的配置项，为回滚功能提供数据
    def get_history(self,req):
        if req.get_history_record:
            filename = os.popen('rospack find xbot_talker').read().strip() + '/defaultconfig/json/dialogue.json'
            with open(filename, 'r') as f:
                data = json.load(f)
            dialogues = data['Dialogue']
            dialogue_list = dialogue_ros()
            for i in range(10):
                tem = dialogue()
                tem.keyword = dialogues[-10+i]['keyword']
                tem.answer = dialogues[-10+i]['answer']
                tem.update_time = dialogues[-10+i]['update_time']
                tem.action = dialogues[-10+i]['action']
                tem.action_mode = dialogues[-10+i]['action_mode']
                dialogue_list.dialogue_ros.append(tem)
            rospy.loginfo(dialogue_list)
            return history_recordResponse(dialogue_list)
        else:
            rospy.logwarn("error: If you want to get the dialogue history record, please set the get_history_record True.")


    # 对话配置回滚功能，根据配置时间可选择回滚到之前的设定
    # 返回值0：成功；１：失败
    def roolback_dialogue(self,req):
        filename = os.popen('rospack find xbot_talker').read().strip() + '/defaultconfig/json/dialogue.json'
        with open(filename, 'r') as f:
            data = json.load(f)
        dialogues = data['Dialogue']
        config_status = 1
        for i in range(len(dialogues)):
            if req.version_info == dialogues[i]['update_time']:
                config_status = 0
                break
        if config_status == 1:
            rospy.logwarn("False: roolback_dialogue failed!")
            return roolback_dialogueResponse(config_status)
        # 回退dialogue.json
        del dialogues[i+1:]
        data['Dialogue'] = dialogues 
        with codecs.open(filename,'w','utf-8') as file:
			json.dump(data, file, indent=4, ensure_ascii=False)
        # 回退grammar.bnf
        filename_bnf = os.popen('rospack find xbot_talker').read().strip() + '/defaultconfig/grammar.bnf'
        with open(filename_bnf,'r') as f:
            data_bnf = f.read()
        roolback_keyword = dialogues[i]['keyword'].encode('utf-8')
        content_bnf = data_bnf.rfind(roolback_keyword)
        if content_bnf != -1:
            data_bnf = data_bnf[:content_bnf+len(roolback_keyword)]+';'
            print(data_bnf)
        with open(filename_bnf,'w') as f:
            f.write(data_bnf)

        # 重新构建语法使回滚操作生效
        home_dir = "/home"
        build_grammar_command = 'roslaunch xbot_talker build_grammar.launch'
        build_grammar = subprocess.Popen(build_grammar_command, shell=True,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            cwd=home_dir)
        rospy.sleep(1)

        # 回退answer_dic.csv
        filename_csv = os.popen('rospack find xbot_talker').read().strip() + '/defaultconfig/answer_dic.csv'
        with open(filename_csv,'r') as f:
            data_csv = csv.reader(f)
            new_data_csv = []
            for item in data_csv:
                new_data_csv.append(item)
                if item[0]==roolback_keyword:
                    # roolback_flag = data_csv.line_num # 可用来查找行
                    break
        with open(filename_csv,'w') as f:
            writer = csv.writer(f)
            writer.writerows(new_data_csv)

        # 重新加载answer_dic.csv使回滚操作生效
        self.nlp_config("back_config","back_config",0,0)
        rospy.loginfo("Successful: roolback_dialogue succeed!")
        return roolback_dialogueResponse(config_status)


    def shutdown(self):
        while rospy.is_shutdown:
            rospy.loginfo("Stopping ...")
        rospy.sleep(1)
        os._exit(0)


if __name__ == '__main__':
    DialogueConfig()
