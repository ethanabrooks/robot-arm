#!/usr/bin/python
 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib.msg 
from control_msgs.msg import FollowJointTrajectoryActionGoal
## END_SUB_TUTORIAL

from std_msgs.msg import String

def move_group_python_interface_tutorial():
  print "============ Initializing node"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('trajectory_planner', anonymous=True)
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("manipulator")
  print "============ Reference frame: %s" % group.get_planning_frame()
  print "============ Reference frame: %s" % group.get_end_effector_link()
  print "============ Robot Groups:"
  print robot.get_group_names()
  # print "============ Printing robot state"
  # print robot.get_current_state()
  print "============"

  print "============ Generating plan"
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.x = -0.70560873414
  pose_target.orientation.y = -0.70856075458
  pose_target.orientation.z = 0.005394901544
  pose_target.orientation.w = 0.005372746147
  pose_target.position.x = 0.81492282360
  pose_target.position.y = 0.19482948259
  pose_target.position.z = -0.0174091126
  group.set_pose_target(pose_target)

  plan = group.plan()

  print "============ plan...", 
  print plan

  print "============ pose...", 
  print group.get_current_pose().pose
  
  # convert plan to trajectory_goal_msg
  id_gen = actionlib.GoalIDGenerator()

  # initialize msg
  jtg = FollowJointTrajectoryActionGoal(goal_id=id_gen.generate_ID())
  
  # header
  jtg.header.seq = 0
  jtg.header.stamp.secs = 0
  jtg.header.stamp.nsecs = 0
  jtg.header.frame_id = ''  # TODO: check this!
  
  # FollowJointTrajectoryGoal
  jtg.goal.trajectory = plan
  jtg.goal.path_tolerance = []
  jtg.goal.goal_tolerance = []

  group.clear_pose_targets()

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
  print "============ STOPPING"


if __name__=='__main__':
    move_group_python_interface_tutorial()

