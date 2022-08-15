import time
from enum import Enum
from dynamixel_sdk import *
import numpy as np
import math
import ikt as Ikt

MOVING_STATUS_THRESHOLD = 8

class ServoRegisterAddress(Enum):
     # list of registers (address,length(bytes))
    DRIVE_MODE           = 10
    OPERATING_MODE       = 11
    PWM_LIMIT            = 36
    MAX_POSITION_LIMIT   = 48
    MIN_POSITION_LIMIT   = 52
    TORQUE_ENABLE        = 64
    LED_ENABLE           = 65
    ERROR_STATUS         = 70
    GOAL_POSITION        = 116
    PRESENT_POSITION     = 132
    PROFILE_VELOCITY     = 112
    PROFILE_ACCELERATION = 108

class ServoRegisterLength(Enum):
    # list of registers length(bytes)
    DRIVE_MODE           = 1
    OPERATING_MODE       = 1
    PWM_LIMIT            = 2
    MAX_POSITION_LIMIT   = 4
    MIN_POSITION_LIMIT   = 4
    TORQUE_ENABLE        = 1
    LED_ENABLE           = 1
    ERROR_STATUS         = 1
    GOAL_POSITION        = 4
    PRESENT_POSITION     = 4
    PROFILE_VELOCITY     = 4
    PROFILE_ACCELERATION = 4

class Servo:

    def __init__(self, ID, minPos, maxPos, offset, side):

        self.ID = ID
        self.minPos = minPos
        self.maxPos = maxPos
        self.offset = offset
        self.side = side

class Robot:

    def __init__(self, port, rate):

        self.rate = rate
        self.motors = []

        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(2.0)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
        # Set port baudrate
        if self.portHandler.setBaudRate(self.rate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")

        self.dxl_comm_result = ''
        self.dxl_error = ''

    def addMotor(self, ID, minPos, maxPos, offset, side):
        
        self.motors.append(Servo(ID, minPos, maxPos, offset, side))

        self.writeParametr(ID, ServoRegisterAddress.MIN_POSITION_LIMIT, int(minPos/360*4095))
        self.writeParametr(ID, ServoRegisterAddress.MAX_POSITION_LIMIT, int(maxPos/360*4095))

    def on(self):

        for i in range(len(self.motors)):
            self.writeParametr(i+1, ServoRegisterAddress.TORQUE_ENABLE, 1)

    def off(self):

        for i in range(len(self.motors)):
            self.writeParametr(i+1, ServoRegisterAddress.TORQUE_ENABLE, 0)

        self.portHandler.closePort()

    def closePort(self):

        self.portHandler.closePort()

    def writeParametr(self, ID, cmd, value):
        addr = cmd.value
        byteLen = ServoRegisterLength[cmd.name].value
    
        dxl_comm_result = COMM_SUCCESS
        dxl_error = 0
        if byteLen == 1:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, ID, addr, value)
        elif byteLen == 2:
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID, addr, value)
        elif byteLen == 4:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, ID, addr, value)
        else:
            dxl_comm_result = COMM_TX_ERROR
            dxl_error = 128

        return dxl_comm_result, dxl_error

    def readParametr(self, ID, cmd):
        addr = cmd.value
        byteLen = ServoRegisterLength[cmd.name].value
        dxl_comm_result = COMM_SUCCESS
        dxl_error = 0
        dxl_comm_value = 0
        if byteLen == 1:
            dxl_comm_value, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, ID, addr)
        elif byteLen == 2:
            dxl_comm_value, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ID, addr)
        elif byteLen == 4:
            dxl_comm_value, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, ID, addr)
        else:
            dxl_comm_result = COMM_RX_CORRUPT
            dxl_error = 128

        return dxl_comm_value, dxl_comm_result, dxl_error

    def setVelocityAndAcc(self, velocity, acceleration):
        
        for i in range(len(self.motors)):
            self.writeParametr(self.motors[i].ID, ServoRegisterAddress.PROFILE_VELOCITY, velocity)
            self.writeParametr(self.motors[i].ID, ServoRegisterAddress.PROFILE_ACCELERATION, acceleration)

    def writePos(self, ID, VALUE):

        if(self.motors[ID-1].side == False):
            VALUE = -VALUE

        VALUE = int((VALUE+self.motors[ID-1].offset)*4095/360)
        dxl_comm_result, dxl_error = self.writeParametr(ID, ServoRegisterAddress.GOAL_POSITION, VALUE)

        return dxl_comm_result, dxl_error

    def setOnePos(self, ID, VALUE):

        dxl_comm_result, dxl_error = self.writePos(ID, VALUE)

        while 1:

            dxl_comm_value, dxl_comm_result, dxl_error = self.readPosition(ID)

            if(abs(dxl_comm_value - VALUE) < MOVING_STATUS_THRESHOLD):
                break

        return dxl_comm_result, dxl_error


    def readPosition(self, ID):

        dxl_comm_value, dxl_comm_result, dxl_error = self.readParametr(ID, ServoRegisterAddress.PRESENT_POSITION)
        dxl_comm_value = int((dxl_comm_value/4095*360-self.motors[ID-1].offset))

        if(self.motors[ID-1].side == False):
            dxl_comm_value = -dxl_comm_value

        return dxl_comm_value, dxl_comm_result, dxl_error

    def setAllPos(self, positions, threeshold):

        for i in range(len(self.motors)-1):
            self.writePos(i+1, positions[i])

        while 1:

            is_ok = 1

            for i in range(len(self.motors)-1):

                dxl_comm_value, dxl_comm_result, dxl_error = self.readPosition(i+1)

                if( abs(dxl_comm_value - positions[i]) > threeshold ):
                    is_ok = 0

            if(is_ok == 1):
                break

        return dxl_comm_result, dxl_error

class Krabopulator(Robot):

    def __init__(self, port, rate):

        super().__init__(port, rate)

        self.addMotor(1, 0,   360, 108, True)
        self.addMotor(2, 0,   188, 180, True)
        self.addMotor(3, 91,  269, 92,  True)
        self.addMotor(4, 0,   360, 180, True)
        self.addMotor(5, 80, 335, 180, True)
        self.addMotor(6, 0,   360, 180, True)
        self.addMotor(7, 105, 133, 105, True) # chwytak

        self.setVelocityAndAcc(150, 7)

    def home(self, q):

        self.setOnePos(3, 45)
        self.setOnePos(2, q[1])
        self.setOnePos(6, q[5])
        self.setOnePos(5, q[4])
        self.setOnePos(4, q[3])
        self.setOnePos(3, q[2])
        self.setOnePos(1, q[0])
        time.sleep(0.5)



ikt = Ikt.CInverseKinematics()
manipulator = Krabopulator('COM4', 57600)
manipulator.on()

time.sleep(2)

for i in range(15):
    ikt.setEndEffectorConfiguration(-150,300,300,90,180,-90+i*9)
    print(ikt.q)
    manipulator.setAllPos(ikt.q, 40)

ikt.setEndEffectorConfiguration(-150,300,300,90,180,-90)
manipulator.setAllPos(ikt.q, 8)

ikt.setEndEffectorConfiguration(-150,300,600,90,180,-90)
manipulator.setAllPos(ikt.q, 8)

ikt.setEndEffectorConfiguration(150,300,300,90,180,-90)
manipulator.setAllPos(ikt.q, 8)

manipulator.setOnePos(7, 20)

for i in range(18):
    ikt.setEndEffectorConfiguration(-300+i*30,300,300,90,180,-90)
    print(ikt.q)
    manipulator.setAllPos(ikt.q, 40)

manipulator.setOnePos(7, 0)

for i in range(20):
    ikt.setEndEffectorConfiguration(-150,300-i*30,300,90,180,-90)
    print(ikt.q)
    manipulator.setAllPos(ikt.q, 40)

ikt.setEndEffectorConfiguration(-150,300,300,90,180,-90)
manipulator.setAllPos(ikt.q, 8)

home = [0, -8, 0, 0, 0, 0]
manipulator.home(home)
manipulator.off()
