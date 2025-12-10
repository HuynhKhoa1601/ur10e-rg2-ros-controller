#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

def unity_pose_callback(msg):
    # Cube marker
    cube_marker = Marker()
    cube_marker.header.frame_id = "world"
    cube_marker.header.stamp = rospy.Time.now()
    cube_marker.ns = "cube_marker"
    cube_marker.id = 0
    cube_marker.type = Marker.CUBE
    cube_marker.action = Marker.ADD
    cube_marker.pose = msg.pose
    cube_marker.scale.x = 0.05
    cube_marker.scale.y = 0.05
    cube_marker.scale.z = 0.05
    cube_marker.color.r = 1.0
    cube_marker.color.g = 0.0
    cube_marker.color.b = 0.0
    cube_marker.color.a = 1.0

    # Pre-grasp marker (slightly above cube)
    pre_grasp_marker = Marker()
    pre_grasp_marker.header.frame_id = "world"
    pre_grasp_marker.header.stamp = rospy.Time.now()
    pre_grasp_marker.ns = "pre_grasp_marker"
    pre_grasp_marker.id = 1
    pre_grasp_marker.type = Marker.SPHERE
    pre_grasp_marker.action = Marker.ADD
    pre_grasp_marker.pose = msg.pose
    pre_grasp_marker.pose.position.z += 0.1  # 10 cm above cube
    pre_grasp_marker.scale.x = 0.04
    pre_grasp_marker.scale.y = 0.04
    pre_grasp_marker.scale.z = 0.04
    pre_grasp_marker.color.r = 0.0
    pre_grasp_marker.color.g = 1.0
    pre_grasp_marker.color.b = 0.0
    pre_grasp_marker.color.a = 1.0

    # Grasp marker (at cube)
    grasp_marker = Marker()
    grasp_marker.header.frame_id = "world"
    grasp_marker.header.stamp = rospy.Time.now()
    grasp_marker.ns = "grasp_marker"
    grasp_marker.id = 2
    grasp_marker.type = Marker.CYLINDER
    grasp_marker.action = Marker.ADD
    grasp_marker.pose = msg.pose
    grasp_marker.scale.x = 0.03
    grasp_marker.scale.y = 0.03
    grasp_marker.scale.z = 0.1
    grasp_marker.color.r = 0.0
    grasp_marker.color.g = 0.0
    grasp_marker.color.b = 1.0
    grasp_marker.color.a = 1.0

    # Publish all markers
    pub.publish(cube_marker)
    pub.publish(pre_grasp_marker)
    pub.publish(grasp_marker)

if __name__ == "__main__":
    rospy.init_node('unity_cube_markers')
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.Subscriber('/unity/cube_pose', PoseStamped, unity_pose_callback)
    rospy.spin()

