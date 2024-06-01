from serialRS485 import RS485Bus
import time

ForceBench = RS485Bus(name='Motor1',device = '/dev/ttyUSB0')

while 1:
    try:

        inputStr = input("Type in command: ")
        strlist = inputStr.split('=')
        strlist[0] = strlist[0].lower()
        st = time.time()
        for i in range(100):
            ForceBench.ReadDistance(ID= 1)
            ForceBench.ReadDistance(ID= 2)
        st2 = time.time()
        timesp = st2-st
        print('Time used:',end = ' ')
        print(timesp)
        # if len(strlist) == 2:
        #     if strlist[0].strip() == 'move':
        #         num = float(strlist[1])
        #         ForceBench.ReadState(positionMM=num)
        #     if strlist[0].strip() == 'movet':
        #         num = float(strlist[1])
        #         ForceBench.MovePositionTurn(position=num)
        #     elif strlist[0].strip() == 'vel_limit':
        #         num = float(strlist[1])
        #         ForceBench.SetMaxVelocityMMps(velocity=num)
        #     elif strlist[0].strip() == 'positiont':
        #         num = float(strlist[1])
        #         ForceBench.ToPositionTurn(position=num)
        #     elif strlist[0].strip() == 'position':
        #         num = float(strlist[1])
        #         ForceBench.ToPositionMM(positionMM=num)
        #     elif strlist[0].strip() == 'id':
        #         num = float(strlist[1])
        #         ForceBench.motorAddress = num
        #         print('Motor communication ID changed to ' + str(num))
        # elif len(strlist) == 1: 
        #     if strlist[0].strip() == 'read':
        #         ForceBench.ReadDistance(ID= 1)
        #     elif strlist[0].strip() == 'weight':
        #         ForceBench.ReadWeight()
        #     elif strlist[0].strip() == 'weight2':
        #         ForceBench.ReadWeightSweep()
        #     elif strlist[0].strip() == 'config':
        #         ForceBench.ReadConfig()
        #     elif strlist[0].strip() == 'stop':
        #         ForceBench.Stop()
        #     elif (strlist[0].strip() == 'exit' or strlist[0].strip() == 'quit'):
        #         ForceBench.Quit()
        #         break
        #     elif (strlist[0].strip() == 'sitrep' or strlist[0].strip() == 'state'or strlist[0].strip() == 'scan'):
        #         ForceBench.ReadState()
        #     elif (strlist[0].strip() == 'loop'):
        #         for i in range(50):
        #             ForceBench.ReadState()
        #     elif strlist[0].strip() == 'home':
        #         ForceBench.Homing()
        line = ForceBench.serial.flush
        print('Serial receive: ',end = '')
        for bitt in line:
            print('%#x' % bitt, end=' ')
        print()
    except Exception as e:
        print("!!!!! Warning !!!!! : ", e)
        print('The second argument must be a number!!!')

