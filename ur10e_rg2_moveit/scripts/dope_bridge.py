#!/usr/bin/env python3
import rospy
import socket
from geometry_msgs.msg import PoseStamped

HOST = "0.0.0.0"
PORT = 5005
TOPIC_NAME = "/dope/target_pose"
FRAME_ID = "robot_base_link"

def dope_bridge():
    rospy.init_node("dope_bridge")
    pub = rospy.Publisher(TOPIC_NAME, PoseStamped, queue_size=10)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    rospy.loginfo(f"TCP server listening on {HOST}:{PORT}")

    while not rospy.is_shutdown():
        conn, addr = server.accept()
        rospy.loginfo(f"Connected: {addr}")
        buffer = ""
        with conn:
            while not rospy.is_shutdown():
                data = conn.recv(4096)
                if not data:
                    break
                buffer += data.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    parts = line.strip().split(",")
                    if len(parts) != 7:
                        rospy.logwarn(f"Bad pose: {line.strip()}")
                        continue
                    try:
                        x, y, z = map(float, parts[0:3])
                        qx, qy, qz, qw = map(float, parts[3:7])
                    except Exception as e:
                        rospy.logwarn(f"Parse error: {e}")
                        continue

                    msg = PoseStamped()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = FRAME_ID
                    msg.pose.position.x = x
                    msg.pose.position.y = y
                    msg.pose.position.z = z
                    msg.pose.orientation.x = qx
                    msg.pose.orientation.y = qy
                    msg.pose.orientation.z = qz
                    msg.pose.orientation.w = qw

                    pub.publish(msg)

if __name__ == "__main__":
    dope_bridge()

