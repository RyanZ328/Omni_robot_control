#!/usr/bin/env python
# -*- coding: utf-8 -*-
# written by 陈松斌 in 10.6

import os
import pygame
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

# 全局变量
cmd = Twist()
# 发布机械臂位姿

cmd_pos_pub = rospy.Publisher('/cmd_pos', Twist, queue_size=5)
ctrl_enable_pub = rospy.Publisher('/enable_ctrl', Int16, queue_size=5)


def keyboardLoop():
    # 初始化
    rospy.init_node('tele_op')
    # 发布频率
    rate = rospy.Rate(100)
    pid = os.getpid()
    sudoPassword = 'ryan'
    command = 'renice -10 %d' % pid
    str = os.system('echo %s|sudo -S %s' % (sudoPassword, command))

    print(pygame.init())
    screen = pygame.display.set_mode((200, 10))
    pygame.display.set_caption("keep me on top")

    # 读取按键循环
    enable_msg = Int16()
    pos_msg = Twist()
    while not rospy.is_shutdown():
        #print(pygame.event.get())
        key_list = pygame.key.get_pressed()
        # print(key_list)
        if key_list[pygame.K_c]: # 退出
            pygame.quit()
            exit()
            break
        enable_msg = Int16()
        pos_msg = Twist()
        pos_msg.linear.x = 200
        pos_msg.linear.y = 200
        if key_list[pygame.K_SPACE]: # 使能控制器
            enable_msg.data = 1

        # if key_list[pygame.K_w]:
        #     xspeed += 1
        #     vel = run_vel_
        # if key_list[pygame.K_s]:
        #     xspeed -= 1
        #     vel = run_vel_
        # if key_list[pygame.K_a]:
        #     yspeed += 1
        #     vel = run_vel_
        # if key_list[pygame.K_d]:
        #     yspeed -= 1
        #     vel = run_vel_
        if key_list[pygame.K_q]:
            pos_msg.angular.z  = 20
        if key_list[pygame.K_e]:
            pos_msg.angular.z  = -20
        
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

