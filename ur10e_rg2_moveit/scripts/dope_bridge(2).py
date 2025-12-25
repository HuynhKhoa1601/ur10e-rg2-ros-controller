#!/usr/bin/env python3
import rospy
import socket
from geometry_msgs.msg import PoseStamped

# CONFIG
HOST = '0.0.0.0' # Listen to everyone
PORT = 5005
TOPIC_NAME = '/dope/target_pose'

def dope_bridge():
    rospy.init_node('dope_topic_publisher')
    pub = rospy.Publisher(TOPIC_NAME, PoseStamped, queue_size=10)
    
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    
    print(f"--- BRIDGE LIVE: Waiting for DOPE Model on Port {PORT} ---")
    print(f"--- Publishing to ROS Topic: {TOPIC_NAME} ---")

    while not rospy.is_shutdown():
        try:
            conn, addr = server.accept()
            with conn:
                print(f"[NET] Connected to Device 1: {addr}")
                while not rospy.is_shutdown():
                    data = conn.recv(1024)
                    if not data: break
                    
                    text = data.decode('utf-8').strip()
                    parts = text.split(',')
                    
                    if len(parts) == 7:
                        # Create ROS Message
                        msg = PoseStamped()
                        msg.header.stamp = rospy.Time.now()
                        msg.header.frame_id = "base_link"
                        
                        # XYZ
                        msg.pose.position.x = float(parts[0])
                        msg.pose.position.y = float(parts[1])
                        msg.pose.position.z = float(parts[2])
                        # Quaternion
                        msg.pose.orientation.x = float(parts[3])
                        msg.pose.orientation.y = float(parts[4])
                        msg.pose.orientation.z = float(parts[5])
                        msg.pose.orientation.w = float(parts[6])
                        print(f"[NET] Connected to Device 1: {msg.pose.position.x}")

                        # PUBLISH TO TOPIC
                        pub.publish(msg)
                        # print(f"[PUB] Sent to Topic: {TOPIC_NAME}")
                        
        except Exception as e:
            print(f"Connection Error: {e}")

if __name__ == '__main__':
    dope_bridge()
