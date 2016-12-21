#!/usr/bin/python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib.msg
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray
from control_msgs.msg import FollowJointTrajectoryActionGoal, \
                             FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal
## END_SUB_TUTORIAL

from std_msgs.msg import String

# class GraspController():
    # def Init(self):
        # self.GetNextGraspPoint = 0

    # def GraspPoint(self, msg):
        # self.GetNextGraspPoint = 1
        # print("looking for next grasp point...")

def publish_trajectory(data=None):
    if data is not None:
        print "============ received points from agile_grasp"
        handle = data.markers[0].points[0]
        print "============ end of points received from agile_grasp"

    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")
    pose = group.get_current_pose().pose

    print "============ Current Pose Orientation..."
    print pose.orientation
    print "============ Current Pose Position..."
    print pose.position
    print "============ end of current pose"

    print "============ Reference frame: %s" % group.get_planning_frame()
    print "============ Reference frame: %s" % group.get_end_effector_link()
    print "============ Robot Groups:"
    print robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()

    print "============ Generating plan"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = pose.orientation.x
    pose_target.orientation.y = pose.orientation.y
    pose_target.orientation.z = pose.orientation.z
    pose_target.orientation.w = pose.orientation.w

    pose_target.position.x = handle.x
    pose_target.position.y = handle.y
    pose_target.position.z = handle.z
    # pose_target.position.x = pose.position.x
    # pose_target.position.y = pose.position.y
    # pose_target.position.z = pose.position.z 

    offsets = [.6, 0, -.4]
    for i, offset in enumerate(offsets):
        group.shift_pose_target(axis=i, value=offset)

    group.set_pose_target(pose_target)
    group.set_start_state_to_current_state()
    group.set_goal_orientation_tolerance(0.01)
    group.set_goal_position_tolerance(0.01)
    group.set_planning_time(0.5)
    group.allow_replanning(True)


    while True:
        plan = group.plan().joint_trajectory
        if plan.points:
            break

    print "============ first point in trajectory..."
    print plan.points[0]

    print "============ last point in trajectory..."
    print plan.points[-1]

    print "============ Generating trajectory_goal_msg..."
    goal = FollowJointTrajectoryGoal(trajectory=plan)

    print "============ sending goal..."
    client.send_goal(goal)
    client.wait_for_result()
    print "Goal state: ", client.get_state()
    print "result: ", client.get_result()

    group.clear_pose_targets()

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('trajectory_planner') #, anonymous=True, )
    client = actionlib.SimpleActionClient(
                       '/arm_controller/follow_joint_trajectory',
                       FollowJointTrajectoryAction)

    rospy.Subscriber('/find_grasps/handles_visual', MarkerArray, publish_trajectory)
    print("subscribed to /find_grasps/handles_visual")
    rospy.spin()
    # publish_trajectory()

    ## When finished shut down moveit_commander.
    # moveit_commander.roscpp_shutdown()
    # print "============ STOPPING"

    # graspController = GraspController()
    # graspController.Init()
    # rospy.Subscriber('/find_grasps/enable_grasp', Bool, GraspController.GraspPoint)
