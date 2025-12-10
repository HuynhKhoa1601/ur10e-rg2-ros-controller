#!/usr/bin/env python3
import moveit_commander
import rospy
from geometry_msgs.msg import Pose

rospy.init_node('test_plan_node')
group = moveit_commander.MoveGroupCommander("arm")

pose = Pose()
pose.position.x = 0.4
pose.position.y = 0.2
pose.position.z = 0.1
pose.orientation.w = 1.0

group.set_pose_target(pose)
plan = group.plan()
print(plan)
