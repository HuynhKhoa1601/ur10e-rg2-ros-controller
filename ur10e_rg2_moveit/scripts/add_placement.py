#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_placement_marker():
    rospy.init_node("placement_debug_marker_node", anonymous=True)
    pub = rospy.Publisher("/placement_debug_marker", Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    marker = Marker()
    marker.header.frame_id = "robot_base_link"  # use your ROS world frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "placement"
    marker.id = 0
    marker.type = Marker.CUBE  # Can also use Marker.SPHERE
    marker.action = Marker.ADD

    # Set the position (converted world coordinates)
    marker.pose.position.x = 0.310
    marker.pose.position.y = 0.465
    marker.pose.position.z = 0.0

    # Set the orientation (converted world quaternion)
    marker.pose.orientation.x = 0.817
    marker.pose.orientation.y = -0.205
    marker.pose.orientation.z = 0.167
    marker.pose.orientation.w = 0.513

    # Size of the cube
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    # Color (red)
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # Alpha = 1 for opaque

    marker.lifetime = rospy.Duration()  # 0 = forever

    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    publish_placement_marker()

