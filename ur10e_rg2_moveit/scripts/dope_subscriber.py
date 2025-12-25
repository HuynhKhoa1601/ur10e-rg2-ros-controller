#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf.transformations import quaternion_matrix
import threading
import time



FRAME_ROBOT = "robot_base_link"

# Plank / workspace parameters
BOARD_QUAT = [0.831615, -0.296885, 0.167739, 0.438338]
BOARD_ROT = quaternion_matrix(BOARD_QUAT)[:3, :3]
PLANK_CENTER_CAM = [0.11353, 0.14637, 1.18553]
PLANK_WIDTH_X = 0.5
PLANK_DEPTH_Y = 0.5
TABLE_CENTER_Z = -0.32
TABLE_HEIGHT = 0.64
CUBE_HEIGHT = 0.05
SAFETY_MARGIN = 0.01
WORKSPACE_MIN_X, WORKSPACE_MAX_X = -1.0, 1.0
WORKSPACE_MIN_Y, WORKSPACE_MAX_Y = -1.0, 1.0

def monitor_threads():
    while not rospy.is_shutdown():
        main_thread = threading.main_thread()
        background_threads = [t for t in threading.enumerate() if t != main_thread]
        print(f"[Monitor] Main thread alive: {main_thread.is_alive()}")
        for t in background_threads:
            print(f"[Monitor] Thread {t.name} alive: {t.is_alive()}")
        # Optional: print stack of main thread to see if it’s stuck
        frame = sys._current_frames()[main_thread.ident]
        print(f"[Monitor] Main thread current function: {frame.f_code.co_name}")
        time.sleep(2)

def callback(msg):
    rospy.loginfo("Sucessfully sent")
    obj_pos_cam = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    
    # Relative to plank
    cube_rel = obj_pos_cam - np.array(PLANK_CENTER_CAM)
    cube_rel_rotated = BOARD_ROT.T.dot(cube_rel)
    cube_x_norm = max(0.0, min(1.0, (cube_rel_rotated[0] + PLANK_WIDTH_X/2) / PLANK_WIDTH_X))
    cube_y_norm = max(0.0, min(1.0, (cube_rel_rotated[1] + PLANK_DEPTH_Y/2) / PLANK_DEPTH_Y))
    
    
    
    # Map to robot workspace
    robot_x = WORKSPACE_MIN_X + cube_x_norm * (WORKSPACE_MAX_X - WORKSPACE_MIN_X)
    robot_y = WORKSPACE_MIN_Y + cube_y_norm * (WORKSPACE_MAX_Y - WORKSPACE_MIN_Y)
    robot_z = TABLE_CENTER_Z + TABLE_HEIGHT / 2 + CUBE_HEIGHT / 2 + SAFETY_MARGIN
    
     # ------------------------ REMOVE PREVIOUS CUBE -------------------------
    remove_co = CollisionObject()
    remove_co.id = "detected_cube"  # same ID
    remove_co.header.stamp = rospy.Time.now()
    remove_co.header.frame_id = "robot_base_link"
    remove_co.operation = CollisionObject.REMOVE
    planning_scene_pub.publish(remove_co)
    rospy.sleep(0.05)

    # Publish Marker
    marker = Marker()
    marker.header.frame_id = FRAME_ROBOT
    marker.header.stamp = rospy.Time.now()
    marker.ns = "detected_object"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = msg.pose
    marker.pose.position.x = robot_x
    marker.pose.position.y = robot_y
    marker.pose.position.z = robot_z
    marker.scale.x = CUBE_HEIGHT
    marker.scale.y = CUBE_HEIGHT
    marker.scale.z = CUBE_HEIGHT
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    marker_pub.publish(marker)

    # Publish CollisionObject
    co = CollisionObject()
    co.id = "detected_cube"
    co.header.stamp = rospy.Time.now()
    co.header.frame_id = FRAME_ROBOT
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [CUBE_HEIGHT, CUBE_HEIGHT, CUBE_HEIGHT]
    co.primitives.append(box)
    co.primitive_poses.append(marker.pose)
    co.operation = CollisionObject.ADD
    
    rospy.loginfo("Mapped cube to robot workspace: x=%.3f y=%.3f z=%.3f", robot_x, robot_y, robot_z)

    planning_scene_pub.publish(co)

def obstacle_listener():
    rospy.Subscriber("/dope/target_pose", PoseStamped, callback)
    rospy.spin()  # keeps subscriber alive in its own thread

def listener():
    global marker_pub, planning_scene_pub
    rospy.init_node("dope_subscriber", anonymous=True)
    marker_pub = rospy.Publisher("/detected_object_marker", Marker, queue_size=1)
    planning_scene_pub = rospy.Publisher("/collision_object", CollisionObject, queue_size=1)

    thread = threading.Thread(target=obstacle_listener, daemon=True)
    thread.start()
    
    monitor_thread = threading.Thread(target=monitor_threads, daemon=True)
    monitor_thread.start()

    rospy.loginfo("[dope_subscriber] Obstacle listener started in background thread")

    # Main thread can do other stuff, e.g., just keep alive
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    listener()

