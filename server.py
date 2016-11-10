#! /usr/bin/env python
from __future__ import print_function

import serial
import os

import actionlib
import rospy
from move_arm.msg import *
from std_msgs.msg import String

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

os.sys.path.append('/home/servicerobot3/DynamixelSDK/python/dynamixel_functions_py')             # Path setting
import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library
import dynamixelDefs

class MoveAction(object):
    # create messages that are used eto publish feedback/result
    _feedback = move_arm.msg.MoveFeedback()
    _result = move_arm.msg.MoveResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            move_arm.msg.MoveAction,
            execute_cb=self.execute_cb,
            auto_start=False)  # TODO: can we make this true and get rid of next line?
        print("Start server...")
        self._as.start()
        self.count = 0

    def execute_cb(self, goal):
        print("Execute callback...")
        # helper variables
        num_joints, port, baudrate = (rospy.get_param(param) for param in
                                      ["/num_joints", "/port", "/baudrate"])

        message = ' '.join(map(str, goal.angles)) + '\n'
        assert len(goal.angles) == num_joints, \
            '%s should have %d values (num_joints)' % \
            (message, num_joints)
        rate = rospy.Rate(10)
        success = True

        # TODO: move motors.
        retv = self.set_motors(goal)

        #self._feedback.angles = goal.angles #map(float, feedback.split())
            # open read/write way string buffer

        #ser.write(unicode(angles_string))

        # with serial.Serial(port=port, baudrate=baudrate, timeout=1) as ser:
        #     print("Waiting for Arduino input...")
        #     read = ser.readline()
        #     print("read: " + read)
            # self._feedback.angles = map(float, read.split())

        # publish info to the console for the user
        rospy.loginfo('%s: passing the following angles to arduino: %s' %
                      (self._action_name, message))

        # publish the feedback
        self._as.publish_feedback(self._feedback)

        if success:
            self._result.angles = self._feedback.angles
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    def set_motors(self, goal):
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

        num_joints, devicename, baudrate, ids, res, jointrange = (rospy.get_param(param) for param in
            ["/num_joints", "/port", "/baudrate", "/deviceids", "/encoderres", "/jointrange"])

        devicenameUTF8 = "/dev/ttyUSB0".encode('utf-8')

        # Initialize PortHandler Structs
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        port_num = dynamixel.portHandler(devicenameUTF8)

        # Initialize PacketHandler Structs
        dynamixel.packetHandler()

        dxl_comm_result = COMM_TX_FAIL                              # Communication result
        dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

        dxl_error = 0                                               # Dynamixel error
        dxl_present_position = 0                                    # Present position

        self._feedback.angles = ids

        # Open port
        if dynamixel.openPort(port_num):
            print("Succeeded to open the port!")
        else:
            print("Failed to open the port!")

        # Set port baudrate
        if dynamixel.setBaudRate(port_num, baudrate):
            print("Succeeded to change the baudrate!")
        else:
            print("Failed to change the baudrate!")


        # Enable Dynamixel Torque
        #for DXL_ID in ids:
        for i in range(0, num_joints):
            if ids[i] != -1:
                DXL_ID = ids[i]
                dxl_goal_position = int(goal.angles[i] * res[i] / (jointrange[i][1] - jointrange[i][0]))
                print("moving joint %i", DXL_ID)

                dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
                if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
                elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
                    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
                else:
                    print("Dynamixel %i has been successfully connected", DXL_ID)

                # Write goal position
                dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position)
                if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
                elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
                    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
                else:
                    print("Dynamixel %i write success", DXL_ID)

        # Disable Dynamixel Torque
        dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
        elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
            dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))


        # Close port
        dynamixel.closePort(port_num)


if __name__ == '__main__':
    rospy.init_node('move_arm')
    MoveAction(rospy.get_name())
    rospy.spin()
