#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

TOPIC_IN = "/dope/target_pose"
TOPIC_MARKER = "/dope/target_pose_marker"

# chỉnh kích thước cube marker (m)
CUBE_SIZE = 0.05

pub_marker = None

def publish_marker(pose_msg: PoseStamped):
    """Publish a RViz marker (cube) at the received pose."""
    m = Marker()
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = pose_msg.header.frame_id if pose_msg.header.frame_id else "base_link"
    m.ns = "dope_target"
    m.id = 0
    m.type = Marker.CUBE
    m.action = Marker.ADD

    m.pose = pose_msg.pose
    m.scale.x = CUBE_SIZE
    m.scale.y = CUBE_SIZE
    m.scale.z = CUBE_SIZE

    # màu xanh dương
    m.color.r = 0.0
    m.color.g = 0.2
    m.color.b = 1.0
    m.color.a = 1.0

    # marker sống 0.3s, nếu stream ngắt sẽ tự biến mất
    m.lifetime = rospy.Duration(0.3)

    pub_marker.publish(m)

def cb(msg: PoseStamped):
    p = msg.pose.position
    q = msg.pose.orientation
    rospy.loginfo(
        f"[SUB] /dope/target_pose | frame={msg.header.frame_id} | "
        f"x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} | "
        f"q=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})"
    )

    # publish marker cho RViz
    publish_marker(msg)

def main():
    global pub_marker
    rospy.init_node("dope_target_pose_subscriber", anonymous=True)

    pub_marker = rospy.Publisher(TOPIC_MARKER, Marker, queue_size=1)
    rospy.Subscriber(TOPIC_IN, PoseStamped, cb, queue_size=10)

    rospy.loginfo(f"Listening: {TOPIC_IN}")
    rospy.loginfo(f"Marker out: {TOPIC_MARKER} (RViz: Add -> Marker)")
    rospy.spin()

if __name__ == "__main__":
    main()
