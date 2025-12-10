#!/usr/bin/env python3
import socket
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from tf.transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    euler_from_quaternion
)

HOST = '0.0.0.0'
PORT = 5005

CAM_POS_WORLD = [1.184, 0.391, 0.061]
CAM_QUAT_WORLD = [0.0, 0.0, 0.0, 1.0]


REFERENCE_Z = 0.0    # floor height
CUBE_HEIGHT = 0.5    # marker height in meters


def object_camera_to_world(obj_pos_cam, obj_quat_cam):
    cam_T = quaternion_matrix(CAM_QUAT_WORLD)
    cam_T[0:3, 3] = CAM_POS_WORLD

    obj_T = quaternion_matrix(obj_quat_cam)
    obj_T[0:3, 3] = obj_pos_cam

    world_T = np.dot(cam_T, obj_T)

    world_pos = world_T[0:3, 3]
    world_quat = quaternion_from_matrix(world_T)

    return world_pos, world_quat


def log_orientation(label, quat):
    roll, pitch, yaw = euler_from_quaternion(quat)
    rospy.loginfo(
        f"{label} quaternion: "
        f"x={quat[0]:.3f}, y={quat[1]:.3f}, z={quat[2]:.3f}, w={quat[3]:.3f}"
    )
    rospy.loginfo(
        f"{label} RPY (rad): "
        f"roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}"
    )


def main():
    rospy.init_node("pose_receiver", anonymous=True)

    pose_pub = rospy.Publisher("/detected_object_pose", PoseStamped, queue_size=1)
    marker_pub = rospy.Publisher("/detected_object_marker", Marker, queue_size=1)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    rospy.loginfo("Waiting for pose from AI laptop...")

    conn, addr = server.accept()
    rospy.loginfo(f"🔗 Connected by {addr}")

    data = conn.recv(1024)
    conn.close()
    server.close()

    text = data.decode('utf-8').strip()
    rospy.loginfo(f"RAW (camera frame): {text}")

    parts = text.split(',')
    if len(parts) != 7:
        rospy.logerr("Invalid pose format")
        return

    # -------- Camera frame --------
    obj_pos_cam = list(map(float, parts[:3]))
    obj_quat_cam = list(map(float, parts[3:]))

    log_orientation("Camera orientation", obj_quat_cam)

    # -------- World frame --------
    world_pos, world_quat = object_camera_to_world(obj_pos_cam, obj_quat_cam)

    # ✅ Correct Z for floor (or table) reference
    world_pos[2] = REFERENCE_Z + CUBE_HEIGHT / 2

    rospy.loginfo(
        f"World position (RViz): "
        f"x={world_pos[0] - 1:.3f}, y={world_pos[1]:.3f}, z={world_pos[2]:.3f}"
    )
    log_orientation("World orientation", world_quat)

    # -------- PoseStamped --------
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "robot_base_link"
    pose_msg.pose.position.x = world_pos[0] - 1
    pose_msg.pose.position.y = world_pos[1]
    pose_msg.pose.position.z = world_pos[2]  # center of cube
    pose_msg.pose.orientation.x = world_quat[0]
    pose_msg.pose.orientation.y = world_quat[1]
    pose_msg.pose.orientation.z = world_quat[2]
    pose_msg.pose.orientation.w = world_quat[3]

    pose_pub.publish(pose_msg)

    # -------- BLUE cube Marker --------
    marker = Marker()
    marker.header.frame_id = "robot_base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "detected_object"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    marker.pose = pose_msg.pose

    # 🔹 Cube size
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = CUBE_HEIGHT

    # 🔹 BLUE color
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    marker.lifetime = rospy.Duration(0)  # forever
    marker_pub.publish(marker)

    rospy.loginfo("✅ Blue cube marker published in RViz")
    rospy.sleep(0.5)


if __name__ == "__main__":
    main()

