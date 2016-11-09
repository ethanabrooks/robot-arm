#! /usr/bin/env python
from __future__ import print_function

import serial

import actionlib
import rospy
from move_arm.msg import *


class MoveAction(object):
    # create messages that are used to publish feedback/result
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

        self._feedback.angles = map(float, feedback.split())

        # publish info to the console for the user
        rospy.loginfo('%s: passing the following angles to arduino: %s' %
                      (self._action_name, message))

        # publish the feedback
        self._as.publish_feedback(self._feedback)

        if success:
            self._result.angles = self._feedback.angles
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('move_arm')
    MoveAction(rospy.get_name())
    rospy.spin()
