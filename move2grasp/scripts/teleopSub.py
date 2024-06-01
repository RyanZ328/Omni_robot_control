from serialCAN import SerialCAN
import time

M3508 = SerialCAN(name='RS485-1',device='/dev/ttyUSB0')


def inverseKinematics(front_xSPD,left_ySPD,twist_zSPD):
    WheelBaseRadius = 0.297
    WheelRadius = 0.127/2
    LF_SPD = (front_xSPD-left_ySPD)*0.7071-twist_zSPD*WheelBaseRadius
    RF_SPD = (-front_xSPD-left_ySPD)*0.7071-twist_zSPD*WheelBaseRadius
    RR_SPD = (-front_xSPD+left_ySPD)*0.7071-twist_zSPD*WheelBaseRadius
    LR_SPD = (front_xSPD+left_ySPD)*0.7071-twist_zSPD*WheelBaseRadius
    return []

def inverseDynamics(front_xForce,left_yForce,twist_zTorque):
    WheelBaseRadius = 0.297
    WheelRadius = 0.127/2
    LF_FOR = (front_xForce-left_yForce)*0.7071/2-twist_zTorque/WheelBaseRadius/4
    RF_FOR = (-front_xForce-left_yForce)*0.7071/2-twist_zTorque/WheelBaseRadius/4
    RR_FOR = (-front_xForce+left_yForce)*0.7071/2-twist_zTorque/WheelBaseRadius/4
    LR_FOR = (front_xForce+left_yForce)*0.7071/2-twist_zTorque/WheelBaseRadius/4
    LF_TOR = LF_FOR*WheelRadius
    RF_TOR = RF_FOR*WheelRadius
    RR_TOR = RR_FOR*WheelRadius
    LR_TOR = LR_FOR*WheelRadius
    return [LF_TOR,RF_TOR,RR_TOR,LR_TOR]

def torque2Amps(torque):
    return torque*3

def normalTorqueOutput(torqueList = [0.0,0.0,0.0,0.0]):
    ampsList = [i*3 for i in torqueList]
    M3508.SetTorqueAmps(torqueAmps1 = ampsList[0],torqueAmps2 = ampsList[1],torqueAmps3 = ampsList[2],torqueAmps4 = ampsList[3])
    return

def biasedTorqueOutput(torqueList = [0.0,0.0,0.0,0.0],biasTorque = 0.05):
    ampsList = [i*3 for i in torqueList]
    M3508.SetTorqueAmps(torqueAmps1 = ampsList[0] + biasTorque,
                        torqueAmps2 = ampsList[1] - biasTorque,
                        torqueAmps3 = ampsList[2] + biasTorque,
                        torqueAmps4 = ampsList[3] - biasTorque)
    return


# for item in range(20):
#     M3508.SetTorqueAmps(torqueAmps1 = item/10)
#     time.sleep(0.1)
# for item in range(20):
#     M3508.SetTorqueAmps(torqueAmps1 = -item/10)
#     time.sleep(0.1)

AmpsLIst =  inverseDynamics(front_xForce = 10,left_yForce = 0.0,twist_zTorque = 0.0)
for i in range(10):
    biasedTorqueOutput(torqueList= AmpsLIst)
    time.sleep(0.1)
biasedTorqueOutput(torqueList= AmpsLIst)
