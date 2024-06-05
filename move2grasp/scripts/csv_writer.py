#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16
import time
import os

# pid = os.getpid()
# sudoPassword = 'ryan'
# command = 'renice -10 %d' % pid
# strs = os.system('echo %s|sudo -S %s' % (sudoPassword, command))

class csvWriter():
    def __init__(self):
        rospy.init_node('csvWriter')
        self.csvFreq = 1000
        self.cmd_pos = [0.0 , 0.0 , 0.0]
        self.cmd_vel = [0.0 , 0.0 , 0.0]
        self.cmd_force = [0.0 , 0.0 , 0.0]
        self.sense_pos = [0.0 , 0.0 , 0.0]
        self.motor_current = [0.0 , 0.0 , 0.0 , 0.0]
        rate = rospy.Rate(self.csvFreq)
        self.openfile = 0
        self.csv_msg = 0
        self.init_time = 0.0
        self.cmd_pos_sub = rospy.Subscriber('/cmd_pos', Twist,callback = self.cmd_pos_cb, queue_size=2)
        self.sens_pos_sub = rospy.Subscriber('/laser_distance', Twist,callback = self.sens_pos_cb, queue_size=2)
        self.sens_angle_sub = rospy.Subscriber('/imu_ang_z', Twist,callback = self.sens_angle_cb, queue_size=2)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist,callback = self.cmd_vel_cb, queue_size=2)
        self.cmd_force_sub = rospy.Subscriber('/cmd_force', Twist,callback = self.cmd_force_cb, queue_size=2)
        self.amps_sub = rospy.Subscriber('/amps' , Float32MultiArray,callback = self.motor_current_cb, queue_size=2)
        self.csv_sub = rospy.Subscriber('/csv_write_enable' , Int16,callback = self.csv_state_cb, queue_size=2)
        self.csv_file = None
        while not rospy.is_shutdown():
            if self.openfile == 0 and self.csv_msg == 1:
                self.openfile = 1
                self.init_time = time.time()
                self.filename = time.strftime("%Y_%m_%d_%H_%M_%S.csv",time.localtime(time.time()))
                self.csv_file = open(self.filename,'w')
                self.csv_file.write("time_stamp,cmd_pos_x,cmd_pos_y,cmd_ang_z,sens_pos_x,sens_pos_y,sens_ang_z,amps_LF,amps_RF,amps_RR,amps_LR\n")
                print("Open File" + self.filename)
            if self.openfile == 1:
                cur_time = time.time() - self.init_time
                self.csv_file.write(str(cur_time) +',')
                self.csv_file.write(str(self.cmd_pos[0]) +','+ str(self.cmd_pos[1]) +','+ str(self.cmd_pos[2]) +',')
                self.csv_file.write(str(self.sense_pos[0]) +','+ str(self.sense_pos[1]) +','+ str(self.sense_pos[2]) +',')
                self.csv_file.write(str(self.motor_current[0]) +','+ str(self.motor_current[1]) +','+ str(self.motor_current[2]) +','+ str(self.motor_current[3]))
                self.csv_file.write('\n')
            if self.csv_msg == -1 and self.openfile == 1:
                self.openfile = 0
                self.csv_file.close()
                print("Close File")
            rate.sleep()
        rospy.spin()
        
    def cmd_pos_cb(self,data = Twist()):
        self.cmd_pos = [data.linear.x , data.linear.y , data.angular.z]
        
    def cmd_vel_cb(self,data = Twist()):
        self.cmd_vel = [data.linear.x , data.linear.y , data.angular.z]
        
    def cmd_force_cb(self,data = Twist()):
        self.cmd_force = [data.linear.x , data.linear.y , data.angular.z]
        
    def motor_current_cb(self,data = Float32MultiArray()):
        self.motor_current = data.data
        
    def csv_state_cb(self,data = Int16()):
        self.csv_msg = data.data
        
    def sens_pos_cb(self,data = Twist()):
        self.sense_pos[0] = data.linear.x
        self.sense_pos[1] = data.linear.y
        
    def sens_angle_cb(self,data = Twist()):
        self.sense_pos[2] = data.angular.z

if __name__ == '__main__':
    try:
        csvWriter()
    except rospy.ROSInterruptException:
        pass
