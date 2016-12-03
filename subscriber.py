#! /usr/bin/env python
from __future__ import print_function
from os.path import expanduser

import serial
import os, ctypes

import actionlib
import rospy
from move_arm.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

os.sys.path.append(expanduser('~') + '/DynamixelSDK/python/dynamixel_functions_py')             # Path setting
import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library
#import dynamixelDefs

# Control table address
ADDR_MX_TORQUE_ENABLE       = 24                            # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36

# Protocol version
PROTOCOL_VERSION            = 1                             # See which protocol version is used in the Dynamixel
TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 100                           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000                          # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 10                            # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Data Byte Length
LEN_MX_GOAL_POSITION        = 2
LEN_MX_PRESENT_POSITION     = 2

class SerialComm():
    def Initialize(self):
        # Initialize PortHandler Structs
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows

        self.num_joints, self.devicename, self.baudrate, self.ids, self.res, self.jointrange = (
            rospy.get_param(param) for param in ["/num_joints", "/port", "/baudrate", "/deviceids", "/encoderres", "/jointrange"])

        devicenameUTF8 = "/dev/ttyUSB0".encode('utf-8')

        self.port_num = dynamixel.portHandler(devicenameUTF8)

        # Initialize PacketHandler Structs
        dynamixel.packetHandler()

        # Open port
        if dynamixel.openPort(self.port_num):
            print("Succeeded to open the port!")
        else:
            print("Failed to open the port!")

        # Set port baudrate
        if dynamixel.setBaudRate(self.port_num, self.baudrate):
            print("Succeeded to change the baudrate!")
        else:
            print("Failed to change the baudrate!")

        # Initialize Groupsyncwrite instance
        self.group_num = dynamixel.groupSyncWrite(self.port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

        for i in range(0, self.num_joints):
            if self.ids[i] != -1:
                # enable torque
                DXL_ID = self.ids[i]
                dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)


    def Destruct(self):
        # Disable Dynamixel Torque
        for i in range(0, self.num_joints):
            if self.ids[i] != -1:
                DXL_ID = self.ids[i]
                dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
                if dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION))
                elif dynamixel.getLastRxPacketError(self.port_num, PROTOCOL_VERSION) != 0:
                    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, PROTOCOL_VERSION))


        # Close port
        dynamixel.closePort(self.port_num)


    def MoveMotors(self, msg):
        goal = [(x * 180 / 3.14159) % 360 for x in msg.position]
        #print(goal)

        dxl_comm_result = COMM_TX_FAIL                              # Communication result
        dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

        dxl_error = 0                                               # Dynamixel error
        dxl_present_position = 0                                    # Present position

        print(msg.header)

        for i in range(0, self.num_joints):
            if self.ids[i] != -1:
                DXL_ID = self.ids[i]
                dxl_goal_position = int(goal[i] * self.res[i] / (self.jointrange[i][1] - self.jointrange[i][0]))

                dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(self.group_num, DXL_ID, dxl_goal_position, LEN_MX_GOAL_POSITION)).value

                # Write goal position
                #dynamixel.write2ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position)

                # Syncwrite goal position
                dynamixel.groupSyncWriteTxPacket(self.group_num)
                if dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION))

                # Clear syncwrite parameter storage
                dynamixel.groupSyncWriteClearParam(self.group_num)



if __name__ == '__main__':
    rospy.init_node('move_arm_cont')
    sc = SerialComm()
    sc.Initialize()
    rospy.Subscriber("joint_states", JointState, sc.MoveMotors)
    print("subscribed to /joint_states")
    #MoveAction(rospy.get_name())
    rospy.spin()
    sc.Destruct()
