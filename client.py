#! /usr/bin/env python
import rospy
import actionlib
import serial
from move_arm.msg import *


NUM_JOINTS = 4


def get_torques():
    print("Enter %d torques, separated by spaces: " % NUM_JOINTS)
    try:
        torques = map(float, raw_input().split())
        assert len(torques) == NUM_JOINTS
        return torques
    except (AssertionError, ValueError):
        return get_torques()


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
    torques = get_torques()
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
        print "Joint angles:", ', '.join(map(str, result.angles))
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
