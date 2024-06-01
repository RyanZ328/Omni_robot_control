#!/usr/bin/env python
# -*- coding: utf-8 -*-
# written by 陈松斌 in 10.6

import os
import sys
import tty
import termios
import pygame
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

# 全局变量
cmd = Twist()
# 发布机械臂位姿

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
grasp_pub = rospy.Publisher('/grasp', String, queue_size=2)
arm_reset_pub = rospy.Publisher('arm_reset_topic', String, queue_size=1)

global can_grasp
global can_release



def grasp_status_cp(msg):
    global can_release, can_grasp
    if msg.data == '1':
        can_release = True
    if msg.data == '0' or msg.data == '-1':
        can_grasp = True


grasp_status = rospy.Subscriber('/grasp_status', String, grasp_status_cp, queue_size=2)

def keyboardLoop():
    # 初始化
    rospy.init_node('teleop')
    rate = rospy.Rate(30)
    rateslow = rospy.Rate(60)
    pid = os.getpid()
    sudoPassword = 'spark'
    command = 'renice -10 %d' % pid
    str = os.system('echo %s|sudo -S %s' % (sudoPassword, command))   
    # 高速移动参数 线0.4-0.5 | 角2.5-2.6
    run_vel_, run_an_ = 15, 8
    # 低速移动参数
    low_vel_, low_an_ = 0.1, 0.4
    
    vel, an_vel = run_vel_, run_an_

    global can_release, can_grasp , can_move,allowSHUAI
    can_grasp = True
    can_release = False
    can_move = True
    allowSHUAI = True

    print(pygame.init())
    screen = pygame.display.set_mode((200, 10))
    pygame.display.set_caption("keep me on top")
    

    count_z = 0
    count_x = 0

    # 读取按键循环
    while not rospy.is_shutdown():
        can_move = True
        xspeed, yspeed, turn = 0, 0, 0
        #print(pygame.event.get())
        key_list = pygame.key.get_pressed()
        # print(key_list)
        if key_list[pygame.K_c]: # 退出
            pygame.quit()
            exit()
            break
        vel, an_vel = run_vel_, run_an_
        msg = String()
        if key_list[pygame.K_SPACE] and can_grasp: # 抓取0
            msg.data = 'G0'
            can_grasp = False
        elif key_list[pygame.K_z] and can_grasp: # 抓取快速
            msg.data = 'H0'
            can_grasp = False
        # elif key_list[pygame.K_b] and can_grasp: # 抓取1
        #     msg.data = 'G1'
        #     can_grasp = False
        # elif key_list[pygame.K_g] and can_grasp: # 抓取2
        #     msg.data = 'G2'
        #     can_grasp = False
        elif key_list[pygame.K_1] and can_release: # 直接放下
            msg.data = 'R0' 
            can_release = False
        elif key_list[pygame.K_2] and can_release: # 放下并回位
            msg.data = 'R1'
            can_release = False
        elif key_list[pygame.K_5]: # 机械臂重置
            msg.data = '0'
            arm_reset_pub.publish(msg)
            
        elif key_list[pygame.K_q] and can_grasp: # 回零
            msg.data = 'T'
        elif key_list[pygame.K_j] and can_grasp: # 左绝杀
            msg.data = 'A'
        elif key_list[pygame.K_k] and can_grasp: # 右绝杀
            msg.data = 'AL'
        elif key_list[pygame.K_3] and can_release: # 放3层位置
            msg.data = 'PLA3'
        
        if msg.data:
            grasp_pub.publish(msg)

        if key_list[pygame.K_w]:
            xspeed += 1
            vel = run_vel_
        if key_list[pygame.K_s]:
            xspeed -= 1
            vel = run_vel_
        if key_list[pygame.K_a]:
            yspeed += 1
            vel = run_vel_
        if key_list[pygame.K_d]:
            yspeed -= 1
            vel = run_vel_
        if key_list[pygame.K_q]:
            turn += 1
            an_vel = run_an_
        if key_list[pygame.K_e]:
            turn -= 1
            an_vel = run_an_
        if key_list[pygame.K_LSHIFT]:
            vel, an_vel = low_vel_, low_an_

        # 发送消息
        cmd.linear.x = xspeed * vel
        cmd.linear.y = yspeed * vel
        cmd.angular.z = turn * an_vel
        if key_list[pygame.K_LCTRL]:
            can_move = False
        #     if key_list[pygame.K_d]:
        #         allowSHUAI = False
        #         msg.data = 'SHUAI'
        #         grasp_pub.publish(msg)
        #         rate.sleep()
        #         time.sleep(0.2)
        #         allowSHUAI = True

        if can_move:
            pub.publish(cmd)
        pygame.display.update()
        pygame.event.pump()
        rate.sleep()


if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass

