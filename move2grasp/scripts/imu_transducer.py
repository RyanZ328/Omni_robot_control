#!/usr/bin/env python

import rospy
import serial
from serial.serialutil import STOPBITS_ONE
import serial.tools.list_ports
from geometry_msgs.msg import Twist
import time

class imuTransducer():
    def __init__(self):
        # self.read_distance_frequency = 100
        rospy.init_node('imuTransducer')
        self.imuSerial = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.005)
        self.imu_pub = rospy.Publisher('/imu_ang_z' , Twist, queue_size=2)
        
        self.imu = Twist()
        angle_init = 0.0
        while(1):
            try:
                line = self.imuSerial.readline()
                if len(line)==3:
                    angle100 = line[0]+line[1]*256
                    if angle100 > 32768:
                        angle100 = angle100-65536
                    angle_init = angle100/100.0 # 单位：度
                    time.sleep(0.001)
                    break
            except Exception as e:
                print("!!!!! Warning !!!!! : ", e)
        angle_calib = 0.05
        init_time = time.time()
        print("Calibrating...")
        for i in range(500):
            while(1):
                try:
                    line = self.imuSerial.readline()
                    if len(line)==3:
                        angle100 = line[0]+line[1]*256
                        if angle100 > 32768:
                            angle100 = angle100-65536
                        angle_calib = angle100/100.0 # 单位：度
                        time.sleep(0.001)
                        break
                except Exception as e:
                    print("!!!!! Warning !!!!! : ", e)
        calib_time = time.time() - init_time
        calib_ratio = (angle_calib-angle_init)/calib_time
        print("Calibrated")
        
        # MotorCtrlRate = rospy.Rate(self.read_distance_frequency)
        while not rospy.is_shutdown():
            try:
                line = self.imuSerial.readline()
                if len(line)==3:
                    angle100 = line[0]+line[1]*256
                    if angle100 > 32768:
                        angle100 = angle100-65536
                    angle = angle100/100.0 # 单位：度
                    self.imu.angular.z = angle
                    angle = angle - (time.time() - init_time) * calib_ratio
                    print(angle)
                    self.imu_pub.publish(self.imu)
                    time.sleep(0.001)
            except Exception as e:
                print("!!!!! Warning !!!!! : ", e)
        rospy.spin()

if __name__ == '__main__':
    try:
        imuTransducer()
    except rospy.ROSInterruptException:
        pass
