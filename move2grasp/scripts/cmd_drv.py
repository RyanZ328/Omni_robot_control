#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from serialCAN import SerialCAN
import time

class motorTransceiver():
    def __init__(self):
        self.motorCtrlFreq = 100
        rospy.init_node('motorTransceiver')
        self.M3508 = SerialCAN(name='RS485-1',device='/dev/usb_m3508')
        self.force_sub = rospy.Subscriber('/cmd_force' , Twist, self.cmd_callback, queue_size=2)
        self.amps_pub = rospy.Publisher('/amps' , Float32MultiArray, queue_size=2)
        self.biasTorqueAmps = 0.8
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.ang_z = 0.0
        self.timeout = 0.0
        MotorCtrlRate = rospy.Rate(self.motorCtrlFreq)
        while not rospy.is_shutdown():
            curTime = time.time()
            if (curTime-self.timeout<0.2):
                TorquesList =  self.inverseDynamics(front_xForce = self.vel_x,left_yForce = self.vel_y,twist_zTorque = self.ang_z)
                self.biasedTorqueOutput(torqueList = TorquesList, biasTorqueAmps = self.biasTorqueAmps)
            else:
                self.biasedTorqueOutput(torqueList = [0.0,0.0,0.0,0.0], biasTorqueAmps = self.biasTorqueAmps)
            MotorCtrlRate.sleep()
        rospy.spin()


    def cmd_callback(self,data = Twist()):
        self.vel_x = data.linear.x
        self.vel_y = data.linear.y
        self.ang_z = data.angular.z
        self.timeout = time.time()
        

    def inverseKinematics(self,front_xSPD,left_ySPD,twist_zSPD):
        WheelBaseRadius = 0.297
        WheelRadius = 0.127/2
        LF_SPD = (front_xSPD-left_ySPD)*0.7071-twist_zSPD*WheelBaseRadius
        RF_SPD = (-front_xSPD-left_ySPD)*0.7071-twist_zSPD*WheelBaseRadius
        RR_SPD = (-front_xSPD+left_ySPD)*0.7071-twist_zSPD*WheelBaseRadius
        LR_SPD = (front_xSPD+left_ySPD)*0.7071-twist_zSPD*WheelBaseRadius
        return []

    def inverseDynamics(self,front_xForce,left_yForce,twist_zTorque):
        WheelBaseRadius = 0.297
        WheelRadius = 0.127/2
        LF_FORCE = (front_xForce-left_yForce)*0.7071/2-twist_zTorque/WheelBaseRadius/4
        RF_FORCE = (-front_xForce-left_yForce)*0.7071/2-twist_zTorque/WheelBaseRadius/4
        RR_FORCE = (-front_xForce+left_yForce)*0.7071/2-twist_zTorque/WheelBaseRadius/4
        LR_FORCE = (front_xForce+left_yForce)*0.7071/2-twist_zTorque/WheelBaseRadius/4
        LF_TOR = LF_FORCE*WheelRadius
        RF_TOR = RF_FORCE*WheelRadius
        RR_TOR = RR_FORCE*WheelRadius
        LR_TOR = LR_FORCE*WheelRadius
        return [LF_TOR,RF_TOR,RR_TOR,LR_TOR]

    def torque2Amps(self,torque):
        return torque*3

    def normalTorqueOutput(self,torqueList = [0.0,0.0,0.0,0.0]):
        ampsList = [i*3 for i in torqueList]
        self.M3508.SetTorqueAmps(torqueAmps1 = ampsList[0],torqueAmps2 = ampsList[1],torqueAmps3 = ampsList[2],torqueAmps4 = ampsList[3])
        return

    def biasedTorqueOutput(self,torqueList = [0.0,0.0,0.0,0.0],biasTorqueAmps = 0.05):
        ampsList = [i*3 for i in torqueList]
        self.M3508.SetTorqueAmps(torqueAmps1 = ampsList[0] + biasTorqueAmps,
                            torqueAmps2 = ampsList[1] - biasTorqueAmps,
                            torqueAmps3 = ampsList[2] + biasTorqueAmps,
                            torqueAmps4 = ampsList[3] - biasTorqueAmps)
        biasedAmpsList = [ampsList[0] + biasTorqueAmps,ampsList[1] - biasTorqueAmps,ampsList[2] + biasTorqueAmps,ampsList[3] - biasTorqueAmps]
        amps_msg = Float32MultiArray()
        amps_msg.data = biasedAmpsList
        self.amps_pub.publish(amps_msg)

# for item in range(20):
#     M3508.SetTorqueAmps(torqueAmps1 = item/10)
#     time.sleep(0.1)
# for item in range(20):
#     M3508.SetTorqueAmps(torqueAmps1 = -item/10)
#     time.sleep(0.1)

# AmpsLIst =  inverseDynamics(front_xForce = 10,left_yForce = 0.0,twist_zTorque = 0.0)
# for i in range(10):
#     biasedTorqueOutput(torqueList= AmpsLIst)
#     time.sleep(0.1)
# biasedTorqueOutput(torqueList= AmpsLIst)

if __name__ == '__main__':
    try:
        motorTransceiver()
    except rospy.ROSInterruptException:
        pass
