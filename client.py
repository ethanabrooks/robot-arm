#! /usr/bin/env python
from __future__ import print_function

import rospy
import actionlib
import serial
from move_arm.msg import *

NUM_JOINTS = rospy.get_param('num_joints')
JOINT_RANGE = [(0, 360)] * 6

def get_user_input(joint_range):
    print('Input %d space-separated joint angles between %.2f and %.2f: '
                % tuple([len(JOINT_RANGE)] + list(joint_range)), end='')
    user_input = raw_input()
    try:
        angles = [float(angle)# * math.pi / 180.0
                for angle in user_input.split(',')]
        for angle in angles:
            low, high = joint_range
            assert(low <= angle <= high)
        assert(len(angles) == len(JOINT_RANGE))
    except (ValueError, AssertionError):
        print('"%s" is not a valid input. ' % user_input)
        return get_user_input(joint_range)
    return angles



def move_arm_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    print("Creating client...")
    client = actionlib.SimpleActionClient('move_arm',
                                          move_arm.msg.MoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("Waiting for server...")
    client.wait_for_server()

    # Creates a goal to send to the action server.
    torques = get_user_input((0, 360))
    goal = move_arm.msg.MoveGoal(torques)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('move_arm_client')
        result = move_arm_client()
        print("Joint angles:", ', '.join(map(str, result.angles)))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
