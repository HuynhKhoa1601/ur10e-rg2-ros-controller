#!/usr/bin/env python3
import socket
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_matrix, quaternion_from_matrix

# --- MoveIt imports ---
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

# ------------------------ CONFIG -------------------------
HOST = '0.0.0.0'
PORT = 5005

# Camera is the origin
CAM_POS_WORLD = [0.0, 0.0, 0.0]
CAM_QUAT_WORLD = [0.0, 0.0, 0.0, 1.0]  # no rotation for simplicity

# --- Board orientation quaternion (from camera) ---
BOARD_QUAT = [0.831615, -0.296885, 0.167739, 0.438338]
BOARD_ROT = quaternion_matrix(BOARD_QUAT)[:3, :3]  # 3x3 rotation matrix

# Correct plank center in camera frame
PLANK_CENTER_CAM = [0.11353, 0.14637, 1.18553]

# Plank dimensions (meters)
PLANK_WIDTH_X = 0.5
PLANK_DEPTH_Y = 0.5

# --- TABLE (must match Unity) ---
TABLE_CENTER_Z = -0.32
TABLE_HEIGHT = 0.64

# --- CUBE ---
CUBE_HEIGHT = 0.05
SAFETY_MARGIN = 0.01

# Robot workspace boundaries
WORKSPACE_MIN_X, WORKSPACE_MAX_X = -1.0, 1.0
WORKSPACE_MIN_Y, WORKSPACE_MAX_Y = -1.0, 1.0

# ------------------------ TRANSFORM FUNCTION -------------------------
def object_camera_to_world(obj_pos_cam, obj_quat_cam):
    """Transform cube from camera frame to world frame (camera as origin)."""
    cam_T = quaternion_matrix(CAM_QUAT_WORLD)
    cam_T[0:3, 3] = CAM_POS_WORLD

    obj_T = quaternion_matrix(obj_quat_cam)
    obj_T[0:3, 3] = obj_pos_cam

    world_T = np.dot(cam_T, obj_T)
    world_pos = world_T[0:3, 3]
    world_quat = quaternion_from_matrix(world_T)
    return world_pos, world_quat

# ------------------------ MAIN FUNCTION -------------------------
def main():
    rospy.init_node("plank_cube_mapper_camera_origin", anonymous=True)

    pose_pub = rospy.Publisher("/detected_object_pose", PoseStamped, queue_size=1)
    marker_pub = rospy.Publisher("/detected_object_marker", Marker, queue_size=1)
    planning_scene_pub = rospy.Publisher("/collision_object", CollisionObject, queue_size=1)

    # ------------------------ SOCKET -------------------------
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    rospy.loginfo("Waiting for cube pose from camera...")

    conn, addr = server.accept()
    rospy.loginfo(f"Connected by {addr}")

    data = conn.recv(1024)
    conn.close()
    server.close()

    text = data.decode('utf-8').strip()
    rospy.loginfo(f"RAW (camera frame): {text}")

    parts = text.split(',')
    if len(parts) != 7:
        rospy.logerr("Invalid pose format, expected 7 values")
        return

    idx = 0  # single point mode
    obj_pos_cam = list(map(float, parts[:3]))
    obj_quat_cam = list(map(float, parts[3:]))

    # ------------------------ TRANSFORM -------------------------
    world_pos, world_quat = object_camera_to_world(obj_pos_cam, obj_quat_cam)

    # Relative to plank
    cube_rel = np.array([
        world_pos[0] - PLANK_CENTER_CAM[0],
        world_pos[1] - PLANK_CENTER_CAM[1],
        world_pos[2] - PLANK_CENTER_CAM[2]
    ])

    # Apply board rotation to align with plank axes
    cube_rel_rotated = BOARD_ROT.T.dot(cube_rel)  # transpose for inverse rotation

    cube_rel_x_rot = cube_rel_rotated[0]
    cube_rel_y_rot = cube_rel_rotated[1]
    cube_rel_z_rot = cube_rel_rotated[2]

    rospy.loginfo(f"Cube {idx} relative to plank (rotated): "
                  f"x={cube_rel_x_rot:.3f}, y={cube_rel_y_rot:.3f}, z={cube_rel_z_rot:.3f}")

    # ------------------------ NORMALIZE TO PLANK -------------------------
    cube_x_norm = max(0.0, min(1.0, (cube_rel_x_rot + PLANK_WIDTH_X / 2) / PLANK_WIDTH_X))
    cube_y_norm = max(0.0, min(1.0, (cube_rel_y_rot + PLANK_DEPTH_Y / 2) / PLANK_DEPTH_Y))
    rospy.loginfo(f"Cube {idx} normalized: x_norm={cube_x_norm:.3f}, y_norm={cube_y_norm:.3f}")

    # ------------------------ MAP TO ROBOT WORKSPACE -------------------------
    robot_x = WORKSPACE_MIN_X + cube_x_norm * (WORKSPACE_MAX_X - WORKSPACE_MIN_X)
    robot_y = WORKSPACE_MIN_Y + cube_y_norm * (WORKSPACE_MAX_Y - WORKSPACE_MIN_Y)
    robot_z = TABLE_CENTER_Z + TABLE_HEIGHT / 2 + CUBE_HEIGHT / 2 + SAFETY_MARGIN
    rospy.loginfo(f"Cube {idx} mapped to robot workspace: x={robot_x:.3f}, y={robot_y:.3f}, z={robot_z:.3f}")

    # ------------------------ REMOVE PREVIOUS CUBE -------------------------
    remove_co = CollisionObject()
    remove_co.id = "detected_cube"  # same ID
    remove_co.header.stamp = rospy.Time.now()
    remove_co.header.frame_id = "robot_base_link"
    remove_co.operation = CollisionObject.REMOVE
    planning_scene_pub.publish(remove_co)
    rospy.sleep(0.05)

    # ------------------------ POSE -------------------------
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "robot_base_link"
    pose_msg.pose.position.x = robot_x
    pose_msg.pose.position.y = robot_y
    pose_msg.pose.position.z = robot_z
    pose_msg.pose.orientation.x = world_quat[0]
    pose_msg.pose.orientation.y = world_quat[1]
    pose_msg.pose.orientation.z = world_quat[2]
    pose_msg.pose.orientation.w = world_quat[3]
    pose_pub.publish(pose_msg)

    # ------------------------ RVIZ MARKER -------------------------
    marker = Marker()
    marker.header.frame_id = "robot_base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "detected_object"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = pose_msg.pose
    marker.scale.x = CUBE_HEIGHT
    marker.scale.y = CUBE_HEIGHT
    marker.scale.z = CUBE_HEIGHT
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker_pub.publish(marker)

    # ------------------------ MOVEIT COLLISION OBJECT -------------------------
    co = CollisionObject()
    co.id = "detected_cube"  # same ID
    co.header.stamp = rospy.Time.now()
    co.header.frame_id = "robot_base_link"
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [CUBE_HEIGHT] * 3
    co.primitives.append(box)
    co.primitive_poses.append(pose_msg.pose)
    co.operation = CollisionObject.ADD
    planning_scene_pub.publish(co)
    rospy.sleep(0.1)

# ------------------------ ENTRY POINT -------------------------
if __name__ == "__main__":
    main()

