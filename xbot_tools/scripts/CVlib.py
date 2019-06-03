#!/usr/bin/env python
#coding=utf-8

"""
computer version liberay includes:

1. ordinary least squares techniques: class: SLF def: OLS


Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.
"""

import PyKDL
import numpy

def GetAngle(quat):
 rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
 return rot.GetRPY()[2]


def GoalOrientation(theta):
 orientation = Quaternion()

 if -numpy.pi < theta < -numpy.pi*2.0/3.0:
  orientation.z = -numpy.sin(theta/2.0)
  orientation.w = -numpy.cos(theta/2.0)

 else:
  orientation.z = numpy.sin(theta/2.0)
  orientation.w = numpy.cos(theta/2.0)

 return orientation


# 产生一个朝向当前目标点的角度
def angle_generater(sub_point, pre_point):
 if sub_point.y - pre_point.y > 0:
  if sub_point.x < pre_point.x:
   # print 'sub_point.y-pre_point.y>0 && sub_point.x<pre_point.x'
   angle = math.pi - abs(math.atan((sub_point.y - pre_point.y) / (sub_point.x - pre_point.x)))
  if sub_point.x == pre_point.x:
   # print 'sub_point.y-pre_point.y>0 && sub_point.x==pre_point.x'
   angle = math.pi / 2
  if sub_point.x > pre_point.x:
   # print 'sub_point.y-pre_point.y>0 && sub_point.x==sub_point.x>pre_point.x'
   angle = math.atan((sub_point.y - pre_point.y) / (sub_point.x - pre_point.x))

 if sub_point.y - pre_point.y < 0:
  if sub_point.x < pre_point.x:
   # print 'sub_point.y-pre_point.y<0 && sub_point.x<pre_point.x'
   angle = -(math.pi - abs(math.atan((sub_point.y - pre_point.y) / (sub_point.x - pre_point.x))))
  if sub_point.x == pre_point.x:
   # print 'sub_point.y-pre_point.y<0 && sub_point.x==pre_point.x'
   angle = -math.pi / 2
  if sub_point.x > pre_point.x:
   # print 'sub_point.y-pre_point.y<0 && sub_point.x>pre_point.x'
   angle = -abs(math.atan((sub_point.y - pre_point.y) / (sub_point.x - pre_point.x)))

 if sub_point.y - pre_point.y == 0:
  if sub_point.x > pre_point.x:
   # print 'sub_point.y-pre_point.y==0 && sub_point.x>pre_point.x'
   angle = 0.0
  if sub_point.x < pre_point.x:
   # print 'sub_point.y-pre_point.y==0 && sub_point.x<pre_point.x'
   angle = math.pi

 return angle


class SLF:
 def __init__(self):
  self.define()


 def define(self):
  self.MeanX = 0
  self.MeanY = 0
  self.Size = 0
  self.CoefficientA = 0
  self.CoefficientB = 0
  self.CoefficientC = 0

 def OLS(self, Line):
  """
  * 最小二乘法直线拟合（不是常见的一元线性回归算法）
  * 将离散点拟合为  a x + b y + c = 0 型直线
  * 假设每个点的 X Y 坐标的误差都是符合 0 均值的正态分布的。
  * 与一元线性回归算法的区别：一元线性回归算法假定 X 是无误差的，只有 Y 有误差。
  * 时间复杂度 2n
  """
  self.size = len(Line)
  if self.size < 2:
   self.CoefficientA = 0
   self.CoefficientB = 0
   self.CoefficientC = 0
   return False
  for point in Line:
   self.MeanX += point.pose.position.x
   self.MeanY += point.pose.position.y
  self.MeanX /= self.size
  self.MeanY /= self.size

  Dxx = 0.0
  Dxy = 0.0
  Dyy = 0.0
  for point in Line:
   Dxx = (point.pose.position.x - self.MeanX)**2
   Dxy = (point.pose.position.x - self.MeanX) * (point.pose.position.y - self.MeanY)
   Dyy = (point.pose.position.y - self.MeanY)**2

  lam = lambda x,y,z: ( (x + y) - numpy.sqrt( (x - y)**2 + 4 * z**2) ) / 2.0
  den = numpy.sqrt( Dxy**2 + (lam(Dxx, Dyy, Dxy) - Dxx)**2 )

  if abs(den) < 0.001:
   if abs(Dxx / Dyy -1) < 0.001:
    return False
   else:
    self.CoefficientA = 1
    self.CoefficientB = 0
    self.CoefficientC = - self.MeanY
  else:
   self.CoefficientA = Dxy / den
   self.CoefficientB = (lam(Dxx, Dyy, Dxy) - Dxx) / den
   self.CoefficientC = - self.CoefficientA * self.MeanX - self.CoefficientB * self.MeanY

  return (True, round(self.CoefficientA, 3), round(self.CoefficientB, 3), round(self.CoefficientC, 3))



 def get_similar_lines(self, Coe1, Coe2):
  """
  *     A1 x + B1 y + C1 = 0
  *     A2 x + B2 y + C2 = 0

  *     U = (B1, -A1)
  *     V = (B2, -A2)
  Then two vectors in the Euclidean plane are parallel when they are both perpendicular to the same direction vector UT
  """
  line1_victor = [Coe1[1], -Coe1[0]]
  line2_victor = [Coe2[1], -Coe2[0]]
  victor = line1_victor[1] * line2_victor[0] - line2_victor[1] * line1_victor[0]
  if 0 <= round(victor, 2) <= 0.2:
   return True
  else:
   return False


 def Orientation_line_com(self, Orientation , Coe1):
  a_odom = GetAngle(Orientation)
  if Coe1[1] != 0:
   predict = numpy.tan(a_odom)
   ratio = -Coe1[0]/Coe1[1]
  else:
   predict = numpy.sin(a_odom)
   ratio = Coe1[0]/numpy.sqrt(Coe1[1]**2 + Coe1[0]**2)

  #print 'cvlib:\nratio:', ratio, '\npredict:', predict, '\npredict - ratio:', abs(abs(predict) - abs(ratio))

  if abs(abs(predict) - abs(ratio)) < 0.05:
   return True
  else:
   return False





