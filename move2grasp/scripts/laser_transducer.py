#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from serialRS485 import RS485Bus
import time

class laserTransducer():
    def __init__(self):
        # self.read_distance_frequency = 100
        rospy.init_node('laserTransducer')
        self.LaserSensorBus = RS485Bus(name='RS485-1',device='/dev/usb_laser',timeout=0.0057)
        # self.LaserSensorBus = RS485Bus(name='RS485-1',device='usb_laser',timeout=0.006)
        self.distance_pub = rospy.Publisher('/laser_distance' , Twist, queue_size=2)
        
        self.distance = Twist()
        # MotorCtrlRate = rospy.Rate(self.read_distance_frequency)
        while not rospy.is_shutdown():
            try:
                self.distance.linear.x = self.LaserSensorBus.ReadDistance(ID= 1)
                self.distance.linear.y = self.LaserSensorBus.ReadDistance(ID= 2)
                print(self.distance.linear.x, end = ' ')
                print(self.distance.linear.y)
                self.distance_pub.publish(self.distance)
                time.sleep(0.001)
            except Exception as e:
                print("!!!!! Warning !!!!! : ", e)
        rospy.spin()

if __name__ == '__main__':
    try:
        laserTransducer()
    except rospy.ROSInterruptException:
        pass
