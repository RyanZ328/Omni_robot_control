#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
from serial.serialutil import STOPBITS_ONE
import serial.tools.list_ports
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from pid import PID
import time
import numpy as np
# import pygame

class ChassisControl():
    def __init__(self):
        rospy.init_node('ChassisControl')
        # 控制频率/IMU接收频率
        self.ctrlRate = 500
        # 底盘合力/力矩发布节点
        self.cmd_force_pub = rospy.Publisher('/cmd_force',Twist,queue_size=2)
        # 激光传感器接受
        self.sense_pos_sub = rospy.Subscriber('/laser_distance',Twist,callback=self.sense_pos_sub_cb,queue_size=2)
        # 速度环前馈输入量
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel',Twist,callback=self.cmd_vel_sub_cb,queue_size=2)
        # 位置环输入量
        self.cmd_pos_sub = rospy.Subscriber('/cmd_pos',Twist,callback=self.cmd_pos_sub_cb,queue_size=2)
        # 位置环输入量
        self.sense_imuZ_sub = rospy.Subscriber('/imu_ang_z',Twist,callback=self.sense_imuZ_sub_cb,queue_size=2)
        
        # 控制环路启动
        self.ctrl_enable = 0
        self.ctrl_enable_sub = rospy.Subscriber('/enable_ctrl',Int16,callback=self.enable_sub_cb,queue_size=2)
        # self.imuSerial = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.005)
        self.ZspdController = PID(P=0.2,I=0.02,D=0.3,IntLimit=2) # 待调参
        self.ZposController = PID(P=6.0,I=0.02,D=0.2,IntLimit=5) # 待调参
        self.XspdController = PID(P=0.2,I=0.05,D=0.002,IntLimit=5) # 待调参
        self.XposController = PID(P=5.0,I=0.1,D=0.001,IntLimit=20) # 待调参
        self.YspdController = PID(P=0.2,I=0.05,D=0.002,IntLimit=5) # 待调参
        self.YposController = PID(P=5.0,I=0.1,D=0.001,IntLimit=20) # 待调参
        self.rate = rospy.Rate(self.ctrlRate)
        self.filterBufferSize = 20
        self.force_cmd = Twist()
        self.setZAngle = 0.0
        self.setZAngularSpd = 0.0
        self.senseZAngle = 0.0
        self.lastZAngle = 0.0
        self.ZAngularSpd = 0.0
        self.ZAngularSpdLIst = list()
        # self.ZAngularSpdLIst = [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]
        self.setXPos = 0.0
        self.setYPos = 0.0
        self.setXSpeed = 0.0
        self.setYSpeed = 0.0
        self.senseXPos = 0.0
        self.senseYPos = 0.0
        self.lastXPos = 0.0
        self.lastYPos = 0.0
        self.XSpeed = 0.0
        self.YSpeed = 0.0
        self.avgXSpeed = 0.0
        self.avgYSpeed = 0.0
        self.XSpdLIst = list()
        self.YSpdLIst = list()
        # self.XSpdLIst = [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]
        # self.YSpdLIst = [0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]
        for i in range(self.filterBufferSize):
            self.ZAngularSpdLIst.append(0.0)
            self.XSpdLIst.append(0.0)
            self.YSpdLIst.append(0.0)
        
        self.currentTime = time.time()
        self.lastTime = self.currentTime
        # control loop
        while not rospy.is_shutdown():
            # print(self.senseZAngle,end = ' ')
            # print(self.setZAngle, end = ' ')
            
            
            # Sensor Filter: Calculate average speed in a period of time
            self.currentTime = time.time()
            
            self.ZAngularSpd = (self.senseZAngle - self.lastZAngle)/(self.currentTime-self.lastTime)
            for i in range(self.filterBufferSize - 1):
                self.ZAngularSpdLIst[i] = self.ZAngularSpdLIst[i+1]
            self.ZAngularSpdLIst[self.filterBufferSize - 1] = self.ZAngularSpd
            avgZAngSpd = np.mean(self.ZAngularSpdLIst)
            
            self.XSpeed = (self.senseXPos - self.lastXPos)/(self.currentTime-self.lastTime)
            for i in range(self.filterBufferSize - 1):
                self.XSpdLIst[i] = self.XSpdLIst[i+1]
            self.XSpdLIst[self.filterBufferSize - 1] = self.XSpeed
            avgXSpd = np.mean(self.XSpdLIst)
            
            self.YSpeed = (self.senseYPos - self.lastYPos)/(self.currentTime-self.lastTime)
            for i in range(self.filterBufferSize - 1):
                self.YSpdLIst[i] = self.YSpdLIst[i+1]
            self.YSpdLIst[self.filterBufferSize - 1] = self.YSpeed
            avgYSpd = np.mean(self.YSpdLIst)
            
            self.lastTime = self.currentTime
            self.lastZAngle = self.senseZAngle
            self.lastXPos = self.senseXPos
            self.lastYPos = self.senseYPos
            
            # Control Computation
            self.ZposController.setValue = self.setZAngle
            self.XposController.setValue = self.setXPos
            self.YposController.setValue = self.setYPos
            # See if the control is enabled and laser sensor is in range
            # if self.ctrl_enable == 1 and self.senseXPos != 201 and self.senseYPos != 201:
            if self.ctrl_enable == 1:
                self.ZspdController.setValue = self.ZposController.pidPosition(curValue=self.senseZAngle) + self.setZAngularSpd
                self.force_cmd.angular.z = self.ZspdController.pidPosition(curValue=avgZAngSpd)
                # self.XspdController.setValue = self.XposController.pidPosition(curValue=self.senseXPos) + self.setXSpeed
                # self.force_cmd.linear.x = self.XspdController.pidPosition(curValue=avgXSpd)
                # self.YspdController.setValue = self.YposController.pidPosition(curValue=self.senseYPos) + self.setYSpeed
                # self.force_cmd.linear.y = self.YspdController.pidPosition(curValue=avgYSpd)
                # Control Actuation
                self.cmd_force_pub.publish(self.force_cmd)
                print('Control enabled')
            elif (self.ctrl_enable == 2): # ! Special manual control mode
                self.force_cmd.linear.x = self.setXPos
                self.force_cmd.linear.y = self.setYPos
                self.force_cmd.angular.z = self.setZAngle
                self.cmd_force_pub.publish(self.force_cmd)
                print('Manual control enabled')
            else:
                self.force_cmd.linear.x = 0.0
                self.force_cmd.linear.y = 0.0
                self.force_cmd.angular.z = 0.0
                self.cmd_force_pub.publish(self.force_cmd)
                print('Control disabled')
                
            self.rate.sleep()
        rospy.spin()
        
    def cmd_vel_sub_cb(self,data = Twist()):
        self.setXSpeed = data.linear.x
        self.setYSpeed = data.linear.y
        self.setZAngularSpd = data.angular.z
        pass
    
    def cmd_pos_sub_cb(self,data = Twist()):
        self.setXPos = data.linear.x
        self.setYPos = data.linear.y
        self.setZAngle = data.angular.z
        pass
    
    def sense_pos_sub_cb(self,data = Twist()):
        self.senseXPos = -data.linear.x+200
        self.senseYPos = -data.linear.y+200
        pass
    
    def sense_imuZ_sub_cb(self,data = Twist()):
        self.senseZAngle = data.angular.z
        pass
    
    def enable_sub_cb(self,data = Int16):
        self.ctrl_enable = data.data
        pass
        
        
    def zero_cmd(self):
        self.force_cmd.angular.z = 0
        self.force_cmd.linear.x = 0
        self.force_cmd.linear.y = 0
        self.cmd_force_pub.publish(self.force_cmd)
        


if __name__ == '__main__':
    try:
        cc = ChassisControl()
    except rospy.ROSInterruptException:
        pass

