#!/usr/bin/env python
# -*- coding: utf-8 -*-
# written by 陈松斌 in 10.6

import os
import pygame
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import math
import time

# 全局变量
cmd = Twist()
# 发布机械臂位姿

cmd_pos_pub = rospy.Publisher('/cmd_pos', Twist, queue_size=2)
ctrl_enable_pub = rospy.Publisher('/enable_ctrl', Int16, queue_size=2)
csv_state_pub = rospy.Publisher('/csv_write_enable', Int16, queue_size=2)


def keyboardLoop():
    # 初始化
    rospy.init_node('tele_op')
    # 发布频率
    ratefreq = 250
    rate = rospy.Rate(ratefreq)
    pid = os.getpid()
    sudoPassword = 'ryan'
    command = 'renice -10 %d' % pid
    str = os.system('echo %s|sudo -S %s' % (sudoPassword, command))

    print(pygame.init())
    screen = pygame.display.set_mode((200, 10))
    pygame.display.set_caption("keep me on top")

    # 读取按键循环
    enable_msg = Int16()
    csv_msg = Int16()
    pos_msg = Twist()
    circleAngular_spd_rad_s = math.pi
    init_time = time.time()
    circle_bool = 0
    circle_count = 0
    circle_radius = 100
    while not rospy.is_shutdown():
        #print(pygame.event.get())
        key_list = pygame.key.get_pressed()
        # print(key_list)
        if key_list[pygame.K_p]: # 退出
            pygame.quit()
            exit()
            break
        enable_msg = Int16()
        enable_msg.data = 0
        pos_msg = Twist()
        # pos_msg.linear.x = 0
        # pos_msg.linear.y = 0
        if key_list[pygame.K_SPACE]: # 使能控制器
            enable_msg.data = 1

        if key_list[pygame.K_w]:
            pos_msg.linear.x  = 100
        if key_list[pygame.K_s]:
            pos_msg.linear.x  = -100
        if key_list[pygame.K_a]:
            pos_msg.linear.y  = 100
        if key_list[pygame.K_d]:
            pos_msg.linear.y  = -100
        if key_list[pygame.K_q]:
            pos_msg.angular.z  = 20
        if key_list[pygame.K_e]:
            pos_msg.angular.z  = -20
        if key_list[pygame.K_r]:
            circle_bool = 1
        if key_list[pygame.K_x]:
            circle_bool = 0
        if key_list[pygame.K_c]:
            csv_msg.data = 1
        if key_list[pygame.K_z]:
            csv_msg.data = -1
        if key_list[pygame.K_LSHIFT]: # 使能控制器
            circle_bool = 0
            enable_msg.data = 2
            pos_msg.angular.z = pos_msg.angular.z/2
            pos_msg.linear.x  = pos_msg.linear.x/5
            pos_msg.linear.y  = pos_msg.linear.y/5
        if circle_bool == 1:
            circle_count = circle_count+1
            pos_msg.linear.x = math.sin(circle_count / ratefreq * circleAngular_spd_rad_s) * circle_radius
            pos_msg.linear.y = math.cos(circle_count / ratefreq * circleAngular_spd_rad_s) * circle_radius
        # if circle_bool == 0:
        #     pos_msg.linear.x = math.sin(circle_count / ratefreq * circleAngular_spd_rad_s) * circle_radius
        #     pos_msg.linear.y = math.cos(circle_count / ratefreq * circleAngular_spd_rad_s) * circle_radius
        csv_state_pub.publish(csv_msg)
        ctrl_enable_pub.publish(enable_msg)
        cmd_pos_pub.publish(pos_msg)
        pygame.display.update()
        pygame.event.pump()
        rate.sleep()


if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass

