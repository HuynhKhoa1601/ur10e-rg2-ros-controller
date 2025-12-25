#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

def add_cube_marker():
    rospy.init_node("test_add_cube_marker")

    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
    rospy.sleep(1.0)  # wait for publisher to be ready

    marker = Marker()
    marker.header.frame_id = "robot_base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "cube_marker"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # Cube pose
    marker.pose.position.x = -0.5
    marker.pose.position.y = 0.7
    marker.pose.position.z = 0.06  # cube center at half height
    marker.pose.orientation.w = 1.0

    # Cube size
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    # Cube color (blue)
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    rospy.loginfo("Publishing cube marker...")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    add_cube_marker()

