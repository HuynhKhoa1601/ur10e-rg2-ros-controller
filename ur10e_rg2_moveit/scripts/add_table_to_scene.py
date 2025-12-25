#!/usr/bin/env python
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

rospy.init_node("add_table_node")

scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(2.0)  # wait for MoveIt to initialize

# Create a table collision object
table_pose = PoseStamped()
table_pose.header.frame_id = "robot_base_link"  # or "world" if that's your planning frame
table_pose.pose.orientation.w = 1.0
table_pose.pose.position.x = 0.0
table_pose.pose.position.y = 0.0
table_pose.pose.position.z = -0.25
table_size = (2.0, 2.0, 0.01)  # large thin plate



scene.add_box("table", table_pose, size=table_size)

rospy.loginfo("✅ Table added to MoveIt planning scene at z=0.64 top.")
rospy.spin()

