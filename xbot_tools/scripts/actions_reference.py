#!/usr/bin/env python
#coding=utf-8
""" 2016/04/05

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy,actionlib,getpass,move_reference,std_msgs.msg
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path,Odometry
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker
from math import *

#回航导航入口
def cruise(Plist):
 #Current_point=rospy.wait_for_message("visualization_marker", Marker)
 point_list=Plist
 intial_point=point_list[0]
 point_list.remove(intial_point)
 #point_list.append(intial_point)
 count=0
 multitasks(intial_point,Plist[0])
 print len(Plist)
 while not rospy.is_shutdown():
  for i in range(len(Plist)):
   rospy.loginfo('moving forwarding to %s_st target'%count)
   count+=1
   if i < (len(Plist)-1):
    print 'forward',Plist[i],Plist[i+1]
    multitasks(Plist[i],Plist[i+1])
   else:
    print 'backward'
    Plist.reverse()
    i=0
    #multitasks(Plist[i],Plist[i-1])
  

#多任务执行 #angle by goal && orientation
def multitasks(intial_point,point):
 move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
 move_base.wait_for_server()
 goal = MoveBaseGoal()
 init_point=intial_point

 angle=move_reference.angle_generater(point,init_point)
 
 try:
  goal.target_pose.header.frame_id = pose.header.frame_id
 except:
  goal.target_pose.header.frame_id = 'map'
 goal.target_pose.pose.position=point
 (goal.target_pose.pose.orientation.x,goal.target_pose.pose.orientation.y,goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w)=move_reference.angle_to_quat(angle)
 move_base.send_goal(goal)
 
 finished_within_time = move_base.wait_for_result(rospy.Duration(300)) 
 if not finished_within_time:
  move_base.cancel_goal()
  rospy.loginfo("Timed out achieving goal")
 else:
  state = move_base.get_state()
  if state == GoalStatus.SUCCEEDED:
   rospy.loginfo("Goal succeeded!")
   rospy.loginfo("State:" + str(state))
  else:
   rospy.loginfo('fail to acheive goal')

                  
#单向导航入口 
def go(current_odom,marker_point):
 if len(marker_point.points)==1:
  point_list=marker_point.points
  intial_point=current_odom.pose.pose.position
  task_point=point_list[0]
  tasks(intial_point,task_point)
 else:
  rospy.loginfo('error in number of markers')
  pass
 
#单向导航入口 
def go_single_marker(current_odom,marker_point):
 if len(marker_point.points)==1:
  point_list=marker_point.points
  intial_point=current_odom.pose.pose.position
  task_point=point_list[0]
  plan_tasks(intial_point,task_point)
 else:
  rospy.loginfo('error in number of markers')
  pass
 
#任务执行 #angle by goal && orientation
def tasks(intial_point,point):
 move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
 move_base.wait_for_server()
 goal = MoveBaseGoal()
 init_point=intial_point

 angle=move_reference.angle_generater(point,init_point)
 
 try:
  goal.target_pose.header.frame_id = pose.header.frame_id
 except:
  goal.target_pose.header.frame_id = 'map'
 goal.target_pose.pose.position=point
 (goal.target_pose.pose.orientation.x,goal.target_pose.pose.orientation.y,goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w)=move_reference.angle_to_quat(angle)
 move_base.send_goal(goal)


#任务执行 #angle by plan
def plan_tasks(intial_point,point):
 tasks(intial_point,point)
 rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan',Path, path_callback, point)
 
#不停的规划新的路径
def path_callback(path,point):
 move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
 move_base.wait_for_server()
 path_poses=path.poses
 path_num=len(path_poses)
 #print 'path_distance: ',path_num
 if path_num>100:  
  print 'long path model'
  move_base.cancel_goal()
  print 'cancel old goal and create a new goal'
  new_goal = MoveBaseGoal()
  new_angle=move_reference.angle_generater(path_poses[int(path_num*0.8)].pose.position,path_poses[int(path_num*0.7)].pose.position)
  try:
   new_goal.target_pose.header.frame_id = pose.header.frame_id
  except:
   new_goal.target_pose.header.frame_id = 'map'
  new_goal.target_pose.pose.position=point
  (new_goal.target_pose.pose.orientation.x,new_goal.target_pose.pose.orientation.y,new_goal.target_pose.pose.orientation.z,new_goal.target_pose.pose.orientation.w)=move_reference.angle_to_quat(new_angle)
  move_base.send_goal(new_goal)
 else:
  pass

 
 
#curse默认模式的注册程序
def pre_regist(odom,modle):
 if odom:
  intial=rospy.wait_for_message("odom",Odometry)
  intial_point=PointStamped()
  intial_point.point=intial.pose.pose.position
  intial_point.header.stamp=rospy.Time.now()
  intial_point.header.frame_id='map'
 else:
  pass
 pose_list,pose_dic,poses=[],{},{}
 if modle=='cruise':
  for i in range(3):#default 3
   rospy.loginfo('请在地图上用 publish point 输入第%s个您希望机器人到达的位置'%(i+1))
   pose=rospy.wait_for_message("clicked_point", PointStamped)
   pose_list.append(pose)
   pose_dic={'pose_%s'%i:{'x':pose.point.x,'y':pose.point.y,'z':pose.point.z}}
   poses.update(pose_dic)
   print 'position',1+i,'recieved'

 elif modle=='voice_interface':
  for i in range(10):
   rospy.loginfo('请在地图上用 publish point 输入第%s个您希望机器人到达的位置'%(i+1))
   pose=rospy.wait_for_message("clicked_point", PointStamped)
   pose_list.append(pose)
   pose_dic={'pose_%s'%i:{'x':pose.point.x,'y':pose.point.y,'z':pose.point.z}}
   poses.update(pose_dic)
   print 'position',1+i,'recieved'

 else:
  rospy.loginfo('error unkown module')

# if back to initial
 try:
  pose_list.append(intial_point)
 except:
  pass

# store
 count=getpass.getuser()
 try:
  write=open('/home/%s/xu_slam/src/nav_staff/map/pre_regist_pose.txt'%count,'w')
 except:
  os.makedirs('/home/%s/xu_slam/src/nav_staff/map'%count)
  write=open('/home/%s/xu_slam/src/nav_staff/map/pre_regist_pose.txt'%count,'w')
 write.writelines('%s'%poses)
 write.close()
 tasks(len(pose_list),pose_list)

#默认模式的读取预注册程序
def pre_load():
 rospy.loginfo('检测到预注册的位置')
 rospy.loginfo('读取预设位置')
 count=getpass.getuser()
 read=open('/home/%s/xu_slam/src/nav_staff/map/pre_regist_pose.txt'%count,'r')
 pose=read.readlines()
 read.close()
 poses=eval(pose[0])
 try:
  intial=rospy.wait_for_message("odom",Odometry)
  intial_point=PointStamped()
  intial_point.point=intial.pose.pose.position
  intial_point.header.stamp=rospy.Time.now()
  intial_point.header.frame_id='map'
 except:
  pass
 pose_list=[]
 for i in range(len(poses)):
  default_point=PointStamped()
  default_point.header.frame_id='map'
  default_point.header.stamp=rospy.Time.now()
  default_point.point.x=poses['pose_%s'%i]['x']
  default_point.point.y=poses['pose_%s'%i]['y']
  default_point.point.z=poses['pose_%s'%i]['z']
  default_point.header.seq=i+1
  pose_list.append(default_point)
 pose_list.append(intial_point)
 tasks(len(pose_list),pose_list)


def position_compara(data_1,data_2):
 if round(data_1.x,4)==round(data_2.x,4):
  x_state=True
 else:
  x_state=False
  
 if round(data_1.y,4)==round(data_2.y,4):
  y_state=True
 else:
  y_state=False
  
 if round(data_1.z,4)==round(data_2.z,4):
  z_state=True
 else:
  z_state=False
  
 return (x_state,y_state,z_state)
 
def orientation_compara(data_1,data_2):
 if round(data_1.x,4)==round(data_2.x,4):
  x_state=True
 else:
  x_state=False
  
 if round(data_1.y,4)==round(data_2.y,4):
  y_state=True
 else:
  y_state=False
  
 if round(data_1.z,4)==round(data_2.z,4):
  z_state=True
 else:
  z_state=False
  
 if round(data_1.w,4)==round(data_2.w,4):
  w_state=True
 else:
  w_state=False
  
 return (x_state,y_state,z_state)
 
 
