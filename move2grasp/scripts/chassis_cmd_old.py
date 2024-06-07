#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
from serial.serialutil import STOPBITS_ONE
import serial.tools.list_ports
from geometry_msgs.msg import Twist
from pid import PID
# import pygame

class ChassisControl():
    def __init__(self):
        rospy.init_node('ChassisControl')
        self.cmd_pub = rospy.Publisher('cmd_force',Twist,queue_size=2)
        self.imuSerial = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.005)
        self.controller = PID(P=0.15,I=0,D=0.5)
        self.rate = rospy.Rate(500)
        self.cmd = Twist()
        while not rospy.is_shutdown():
            while(1):
                line = self.imuSerial.readline()
                if len(line)==3:
                    angle = line[0]+line[1]*256
                    if angle > 32768:
                        angle = angle-65536
                    # print(angle)
                    break
            self.cmd.angular.z = self.controller.pidPosition(curValue=angle)
            self.cmd_pub.publish(self.cmd)
            self.rate.sleep()
        rospy.spin()
        
    def zero_cmd(self):
        self.cmd.angular.z = 0
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd_pub.publish(self.cmd)
        


if __name__ == '__main__':
    try:
        cc = ChassisControl()
    except rospy.ROSInterruptException:
        pass

