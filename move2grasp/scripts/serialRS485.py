import serial
from serial.serialutil import STOPBITS_TWO
import serial.tools.list_ports
import struct



class RS485Bus:
    # : None
    # 类名：RS485ForceBench
    # 成员：
    # name:        电机名称，暂无意义
    # comPort:     Windows串口号，字符串，('COM?')
    ## serial:     pyserial串口对象

    def __init__(self, name='DistanceSensor', device='COM-1',Baud = 256000,timeout = 0.006):
        # 初始化电机对象
        # comPort:      可以传参命名为固定串口如'COM3'；若不提前传参，会在控制台终端询问串口号(string)
        # motorName:    电机名称，暂无意义(string)
        # motorAddress: 电机在RS485总线上的地址(int)，默认为1
        # sensorAddress:传感器在RS485总线上的地址(int)，默认为2
        self.name = name
        self.device = device
        self.Baud = Baud
        self.timeout = timeout
        self.RS485_init()

    def RS485_init(self):
        # 初始化电机
        portsName = []
        try:

            port_list = list(serial.tools.list_ports.comports())
            print(port_list)
            if len(port_list) == 0:
                print('No serial port avalible')
                exit(1)
            else:
                for i in range(0, len(port_list)):
                    print(port_list[i])
                    print(port_list[i].device)
                    portsName.append(port_list[i].device)
            while 1:
                if self.device != 'COM-1':
                    break
                inputStr = input(
                    "Type in serial port number: (Example: type in COM5 -> Enter): ")
                inputStr = inputStr.upper().strip()
                for portName in portsName:
                    if inputStr == portName:
                        self.device = inputStr

            self.serial = serial.Serial(self.device, self.Baud, timeout=self.timeout)
            print("Serial Port Info: ", self.serial)

        except Exception as e:
            print("!!!!! Warning !!!!! : ", e)
            exit(1)


    def ReadDistance(self,ID = 2,CRC = True):
        # 读取当前目标位置和实际位置，实时力数值
        signal = [ID, 0x04, 0x00, 0x00, 0x00, 0x02]
        # print('Reading Sensor State')
        if CRC:
            signal = RS485Bus.ModRTU_CRC16(signal=signal)
        self.transmitSignal(signal=signal)
        line = self.serial.read(25)
        distanceMM = -1
        if (len(line)==9):
            # print('RS485 receive: ',end = '')
            # for bitt in line:
            #     print('%#x' % bitt, end=' ')
            distanceUM = line[3]*16777216 + \
                line[4]*65536 + line[5]*256 + line[6]
            if distanceUM >= 435000:
                distanceUM = -1
                distanceMM = -1
                # print('Distance out of range')
            else:
                distanceMM = distanceUM / 1000.0
            # print(distanceMM)
        else:
            print('ID',end=' ')
            print(ID,end=' ')
            print('Read failed')
        return distanceMM
        
    def Quit(self):
        # 切断通信
        self.serial.close()

    def ModRTU_CRC16(signal):
        # ModBusRTU校验码算法函数
        leng=len(signal)
        crc = 0xffff
        crcSeq = []
        for pos in range(leng):
            crc ^= signal[pos]
            crcSeq.append(signal[pos])
            for i in range(8, 0, -1):
                if ((crc & 0x0001) != 0):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        crcSeq.append(crc & 0xff)
        crcSeq.append(crc >> 8)
        return crcSeq

    def transmitSignal(self, signal):
        # print('RS485 transmmit: ', end='')
        # # 串口发送16进制信号
        # for sig in signal:
        #     print('%#x' % sig, end=' ')
        # print()
        if self.serial.writable():
            self.serial.write(signal)

    def floatToBytes(self, f):
        bs = struct.pack("f", f)
        return (bs[3], bs[2], bs[1], bs[0])

    # 以下为称重传感器串口测试函数，非电机控制函数，弃用

    def ReadWeight(self):
        #
        signal = [0x02, 0x03, 0x00, 0x01, 0x00, 0x02]
        print('weight')
        self.transmitSignal(RS485Bus.ModRTU_CRC16(
            signal=signal, len=len(signal)))

    def ReadWeightSweep(self):
        #
        signal = [0x02, 0x03, 0x9C, 0x40, 0x00, 0x02]
        print('weight2')
        self.transmitSignal(RS485Bus.ModRTU_CRC16(
            signal=signal, len=len(signal)))

    def ReadConfig(self):
        #
        signal = [0x02, 0x03, 0x01, 0x64, 0x00, 0x02]
        print('config')
        self.transmitSignal(RS485Bus.ModRTU_CRC16(
            signal=signal, len=len(signal)))

