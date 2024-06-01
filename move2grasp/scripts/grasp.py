#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import random
import ctypes
import roslib
import rospy
import smach
import smach_ros
import threading
import thread
import string
import math
import cv2

import time

import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from spark_carry_object.msg import *


class GraspObject():
    '''
    监听主控，用于物品抓取功能
    '''

    def __init__(self):
        '''
        初始化
        '''

        global xc, yc, xc_prev, yc_prev, found_count,arr,maxvals,AT3,LowerBlue, UpperBlue
        AT3 = False
        self.arm_reset_pub = rospy.Publisher('arm_reset_topic', String, queue_size=1)
        maxvals = [100]*5
        xc = 0
        yc = 0
        xc_prev = xc
        yc_prev = yc
        found_count = 0
        self.is_found_object = False
        # test
        self.sub0 = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb, queue_size=1)
        # 订阅机械臂抓取指令
        self.sub2 = rospy.Subscriber(
            '/grasp', String, self.grasp_cp, queue_size=1)
        # 发布机械臂位置
        self.pub1 = rospy.Publisher(
            'position_write_topic', position, queue_size=10)
        # 发布机械臂吸盘
        self.pub2 = rospy.Publisher('pump_topic', status, queue_size=3)
        # 发布机械臂状态
        self.grasp_status_pub = rospy.Publisher('/grasp_status', String, queue_size=1)
        # 发布TWist消息控制机器人底盘
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pos0 = position()
        self.pos0.x = 105
        self.pos0.y = 0
        self.pos0.z = 25
        #读取机械臂矫正数据
        filename = os.environ['HOME'] + "/thefile.txt"
        file_pix = open(filename, 'r')
        s = file_pix.read()
        file_pix.close()
        arr=s.split()
        LowerBlue, UpperBlue = self.hsv_value()


    def grasp_cp(self, msg):
        if (msg.data[0] == 'G' or msg.data[0] == 'H') :
            # 订阅摄像头话题,对图像信息进行处理
            # self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb, queue_size=1)
            self.is_found_object = False
            rate = rospy.Rate(10)
            times=0
            steps=0
            for i in range(200):
                if(self.is_found_object):
                    break
                rate.sleep()
                if i > 30:
                    # self.sub.unregister()
                    print("stop grasp\n")
                    status=String()
                    status.data='-1'
                    self.grasp_status_pub.publish(status)
                    return


            # while not self.is_found_object:
            #     rate.sleep()
            #     times+=1
            #     # 转一圈没有发现可抓取物体,退出抓取
            #     if steps>=5:
            #         self.sub.unregister()
            #         print("stop grasp\n")
            #         status=String()
            #         status.data='-1'
            #         self.grasp_status_pub.publish(status)
            #         return
            #     # 旋转一定角度扫描是否有可供抓取的物体
            #     if times>=30:
            #         times=0
            #         steps+=1
            #         self.turn_body()
            #         print("not found\n")


            print("unregisting sub\n")
            # self.sub.unregister()
            print("unregisted sub\n")
            # 抓取检测到的物体  
            floor = int(msg.data[1])  
            self.grasp(floor)
            status=String()
            status.data='1'
            self.grasp_status_pub.publish(status) 
        elif msg.data[0] == 'R':
            # 放下物体
            if msg.data[1] == '0':
                self.is_found_object = False
                self.release_objectdirect()
                status=String()
                status.data='0'
                self.grasp_status_pub.publish(status) 
            elif(msg.data[1] == '1'):
                self.is_found_object = False
                floor = int(msg.data[1])
                self.release_object(floor)
                status=String()
                status.data='0'
                self.grasp_status_pub.publish(status) 
        elif msg.data == 'S':
            # 伸出机械臂扫除二层
            self.is_found_object = False
            self.sweep_the_floor()
            status=String()
            status.data='0'
            self.grasp_status_pub.publish(status) 
        elif msg.data == 'T':
            # 回零
            self.tozero()
        elif msg.data == 'ARES':
            # 回零
            self.ARESLOGO()
        elif msg.data == 'A':
            # 回零
            self.JUESHA()
        elif msg.data == 'AL':
            # 回零
            self.JUESHAR()
        elif msg.data == 'PLA3':
            # 回零
            self.place_objectat3()



    def JUESHA(self):
        r1 = rospy.Rate(60) # 0.5s
        poss =  position()
        
        poss.x = 50
        poss.y = 260
        poss.z = -90
        self.pub1.publish(poss)
        r1.sleep()
        time.sleep(1.5)
        msgrr = String()
        msgrr.data = '0'
        self.arm_reset_pub.publish(msgrr)
        r1.sleep()

    
    def JUESHAR(self):
        r1 = rospy.Rate(60) # 0.5s
        poss =  position()
        
        poss.x = 50
        poss.y = -260
        poss.z = -90
        self.pub1.publish(poss)
        r1.sleep()
        time.sleep(1.5)
        msgrr = String()
        msgrr.data = '0'
        self.arm_reset_pub.publish(msgrr)
        r1.sleep()


    def ARESLOGO(self):
        rat1 = rospy.Rate(24)
        filename = os.environ['HOME'] + "/ARESGEqualLength.txt"
        # filename = os.environ['HOME'] + "/ARESG.txt"
        file_pix = open(filename, 'r')
        aresgco = file_pix.readlines()
        pos = position()
        pos.z = 0
        for line in aresgco:                          #依次读取每行  
            arrs = line.split()
            pos.x = float(arrs[0])
            pos.y = float(arrs[1])
            self.pub1.publish(pos)
            msg = String()
            msg = "x" + str(pos.x) + "y" + str(pos.y)
            rospy.loginfo(msg)

            rat1.sleep()

        
        self.pub1.publish(self.pos0)
        file_pix.close()


    # 执行抓取
    def grasp(self, floor):
        print("start to grasp\n")
        global xc, yc, found_count,arr
        # stop function
        a = [0]*2
        b = [0]*2
        a[0]=float(arr[0])
        a[1]=float(arr[1])
        b[0]=float(arr[2])
        b[1]=float(arr[3])
        print('k and b value:',a[0],a[1],b[0],b[1])
        r1 = rospy.Rate(2) # 0.5s
        pos = position()

        # 物体所在坐标+标定误差
        pos.x = a[0] * yc + a[1]
        pos.y = b[0] * xc + b[1]
        pos.z = -50
        self.pub1.publish(pos)
        print("z = -83\n")
        self.pub2.publish(1)
        r1.sleep()

        # 开始吸取物体
        self.pub2.publish(1)
        r1.sleep()

        # 提起物体
        pos.x = 260
        pos.y = 0
        pos.z = 80
        self.pub1.publish(pos)
        r1.sleep()
        msgrr = String()
        msgrr.data = '0'
        self.arm_reset_pub.publish(msgrr)

    def hsv_value(self):
        try:
            filename = os.environ['HOME'] + "/color_block_HSV.txt"
            with open(filename, "r") as f:
                for line in f:
                    split = line.split(':')[1].split(' ')
                    lower = split[0].split(',')
                    upper = split[1].split(',')
                    for i in range(3):
                        lower[i] = int(lower[i])
                        upper[i] = int(upper[i])

            lower = np.array([lower[0]-10,lower[1]-10,lower[2]-10])
            upper = np.array([upper[0]+10,upper[1]+10,upper[2]+10])
        except:
            raise IOError('could not find hsv_value file : {},please execute #13 command automatically '
                          'generate this file'.format(filename))

        return lower, upper


    # 使用CV检测物体       
    def image_cb(self,data):
        global xc, yc, xc_prev, yc_prev, found_count,arr,maxvals, LowerBlue, UpperBlue
        # change to opencv
        try:
            frame = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        seedPoint=(320,479)
        
        # time.sleep(0.03)
        
        oneFound = False
        # change rgb to hsv
        cv_image2 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # cv2.imshow("rawimg", frame)
        mask = cv2.inRange(cv_image2, LowerBlue, UpperBlue)
        cv_image3 = cv2.bitwise_and(cv_image2, cv_image2, mask=mask)
        cv2.imshow("befmask", cv_image3)
        # v value process
        cv_image4 = cv_image3[:, :, 2]
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(cv_image4, mask=mask)
        if(max_val< 110):
            max_val = 110
        maxvals[4] = maxvals[3]
        maxvals[3] = maxvals[2]
        maxvals[2] = maxvals[1]
        maxvals[1] = maxvals[0]
        maxvals[0] = max_val
        max_val = (maxvals[0] + maxvals[1] + maxvals[2] + maxvals[3] + maxvals[4])/5
        LowerBl = np.array([max_val-95])
        UpperBl = np.array([max_val])
        mask = cv2.inRange(cv_image4, LowerBl, UpperBl)
        cv2.imshow("rawblue", mask)
        cv_image4 = mask
        (_, thresh) = cv2.threshold(cv_image4, 70, 255, cv2.THRESH_BINARY)
        cv_image4 = cv2.erode(cv_image4, None, iterations=2)
        # cv2.imshow("erode1", cv_image4)
        im_floodfill = cv_image4.copy()
        # Mask 用于 floodFill，官方要求长宽+2
        h, w = cv_image4.shape[:2]
        mask1 = np.zeros((h+2, w+2), np.uint8)
        # floodFill函数中的seedPoint对应像素必须是背景
        # 得到im_floodfill 255填充非孔洞值
        cv2.floodFill(im_floodfill, mask1,seedPoint, 255)
        # 得到im_floodfill的逆im_floodfill_inv
        im_floodfill_inv = cv2.bitwise_not(im_floodfill)
        # 把cv_image4、im_floodfill_inv这两幅图像结合起来得到前景
        im_out = cv_image4 | im_floodfill_inv
        # 保存结果
        cv_image4 = im_out
        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
        # test = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        # # cv_image5 = cv2.dilate(cv_image5, None, iterations=2)
        # cv_image5 = cv2.erode(cv_image4, None, iterations=8)
        cv2.imshow("processed", cv_image4)
        cv2.waitKey(1)
        _, contours, hier = cv2.findContours(cv_image4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # if find contours, pick the biggest box
        if len(contours) > 0:
            size = []
            size_max = 0
            lenglowest = 1000000
            oneFound = False
            for i, c in enumerate(contours):
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
                y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
                w = math.sqrt((box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2)
                h = math.sqrt((box[0][0] - box[3][0]) ** 2 + (box[0][1] - box[3][1]) ** 2) 
                size.append(w * h)
                # if size[i] > size_max:
                a = [0]*2
                b = [0]*2
                a[0]=float(arr[0])
                a[1]=float(arr[1])
                b[0]=float(arr[2])
                b[1]=float(arr[3])
                armx = a[0] * y_mid + a[1]
                army = b[0] * x_mid + b[1]
                arm_length = armx * armx + army * army
                if arm_length < lenglowest and size[i] > 5000 and arm_length < 95000 and army < 180 and army > -180:
                    lenglowest = y_mid
                    size_max = size[i]
                    index = i
                    xc = x_mid
                    yc = y_mid
                    msg = String()
                    msg = "ylow:" + str(lenglowest) + " size:" + str(size[i]) + "army:" +str(army) + "lens" + str(arm_length)
                    rospy.loginfo(msg)
                    oneFound = True
            # if box is not moving for 20 times
            # print found_count
            if found_count >= 5:
                self.is_found_object = True
                # cmd_vel = Twist()
                # self.cmd_vel_pub.publish(cmd_vel)
            else:
                # if box is not moving
                if abs(xc - xc_prev) <= 5 and abs(yc - yc_prev) <= 5  and oneFound :
                    found_count = found_count + 1
                else:
                    found_count = 0
        else:
            found_count = 0
            self.is_found_object = False
        if (not oneFound):
            found_count = 0
            self.is_found_object = False

        xc_prev = xc
        yc_prev = yc

    def tozero(self):
        r1 = rospy.Rate(10) # 0.1s
        poss =  position()
        
        poss.x = 140
        poss.y = 0
        poss.z = 0
        self.pub1.publish(poss)
        r1.sleep()
        time.sleep(1)
        msgrr = String()
        msgrr.data = '0'
        self.arm_reset_pub.publish(msgrr)
        r1.sleep()

    # 释放物体
    def release_object(self, floor):
        r1 = rospy.Rate(2) # 0.5s
        pos = position()

        # stop pump
        self.pub2.publish(0)

        self.pub1.publish(self.pos0)
        r1.sleep()

        
        msgrr = String()
        msgrr.data = '0'
        self.arm_reset_pub.publish(msgrr)

        
    def release_objectdirect(self):
        r1 = rospy.Rate(5)

        self.pub2.publish(0)
        r1.sleep()

        
        msgrr = String()
        msgrr.data = '0'
        self.arm_reset_pub.publish(msgrr)


        
    def place_objectat3(self):
        r1 = rospy.Rate(1) # 0.5s
        poss =  position()
        
        poss.x = 240
        poss.y = 0
        poss.z = 170
        self.pub1.publish(poss)
        msg = String()
        msg = "PLAAT3"
        rospy.loginfo(msg)
        AT3 = True
        r1.sleep()
        

    def sweep_the_floor(self):
        r1 = rospy.Rate(1.2) # 0.5s
        pos = position()

        pos.x = 250
        pos.y = -150
        pos.z = -10
        self.pub1.publish(pos)
        r1.sleep()

        pos.x = 250
        pos.z = -10
        pos.y = 150
        self.pub1.publish(pos)
        r1.sleep()

        self.pub1.publish(self.pos0)


        # r2.sleep()
        # msgrr = String()
        # msgrr.data = '0'
        # self.arm_reset_pub.publish(msgrr)




        
if __name__ == '__main__':
    try:
        rospy.init_node('GraspObject', anonymous=False)
        rospy.loginfo("Init GraspObject main")   
        GraspObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End spark GraspObject main")

