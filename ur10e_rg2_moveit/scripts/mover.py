#!/usr/bin/env python
from __future__ import print_function

import rospy
import sys
import copy
import time
import moveit_commander

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, CollisionObject
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import PlanningSceneInterface

from ur10e_rg2_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

# Joint names used to build RobotState for MoveIt start states
joint_names = [
    'robot_shoulder_pan_joint', 'robot_shoulder_lift_joint', 'robot_elbow_joint',
    'robot_wrist_1_joint', 'robot_wrist_2_joint', 'robot_wrist_3_joint'
]

# Global planning scene so both callback and main code share it
planning_scene = None


# compatibility helper for MoveIt plan() return type
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan


def wait_for_scene_update(scene, object_name, timeout=4.0):
    """
    Wait until the planning scene knows about object_name (or timeout).
    Returns True if object is present in known object names, else False.
    """
    start = rospy.Time.now()
    rate = rospy.Rate(10)
    while (rospy.Time.now() - start).to_sec() < timeout:
        known = scene.get_known_object_names()
        if object_name in known:
            return True
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
    return False


def plan_trajectory(move_group, destination_pose, start_joint_angles,
                    max_retries=3, initial_timeout=20.0, timeout_increment=5.0):
    """
    Plan a trajectory from start_joint_angles to destination_pose.
    Returns a RobotTrajectory-like object on success or raises Exception on failure.
    """
    # Safe copy so we don't mutate the incoming request
    dest = copy.deepcopy(destination_pose)

    # Build a start state from given joint angles
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)


    move_group.set_pose_target(dest)

    timeout_seconds = initial_timeout
    for attempt in range(max_retries):
        rospy.loginfo(f"[plan_trajectory] attempt {attempt+1}/{max_retries} — planning_time={timeout_seconds}s")
        move_group.set_planning_time(timeout_seconds)
        plan = move_group.plan()

        if plan:
            planned_trajectory = planCompat(plan)
            if hasattr(planned_trajectory, 'joint_trajectory') and planned_trajectory.joint_trajectory.points:
                rospy.loginfo("[plan_trajectory] planning succeeded")
                return planned_trajectory

        rospy.logwarn(f"[plan_trajectory] attempt {attempt+1} failed — increasing timeout and retrying")
        timeout_seconds += timeout_increment

    raise Exception(f"Trajectory could not be planned after {max_retries} attempts. Destination: {dest}, start_joints: {start_joint_angles}")


def log_pose(pose, label="Pose"):
    p = pose.position
    rospy.loginfo(f"{label} - ROS Position: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")


def plan_pick_and_place(req):
    """
    Main service handler: plans pick trajectory then place trajectory.
    Always logs first/last joint positions for pick and place trajectories.
    """
    response = MoverServiceResponse()
    rospy.loginfo("[plan_pick_and_place] Received a request!")

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    rospy.loginfo(f"[plan_pick_and_place] MoveGroupCommander created for group: {group_name}")

    current_robot_joint_configuration = req.joints_input.joints
    rospy.loginfo(f"[plan_pick_and_place] Current joint configuration: {current_robot_joint_configuration}")

    try:
        # --- Pick Trajectory ---
        rospy.loginfo("[plan_pick_and_place] Planning pick trajectory...")
        log_pose(req.pick_pose, label="Pick Pose")

        try:
            pre_grasp_traj = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
            rospy.loginfo(f"[plan_pick_and_place] Pick trajectory points: {len(pre_grasp_traj.joint_trajectory.points)}")
            response.trajectories.append(pre_grasp_traj)

            rospy.loginfo(f"[DEBUG] Pick trajectory first point: {pre_grasp_traj.joint_trajectory.points[0].positions}")
            last_joint_positions = pre_grasp_traj.joint_trajectory.points[-1].positions
            rospy.loginfo(f"[DEBUG] Pick trajectory last point: {last_joint_positions}")

        except Exception as e:
            rospy.logerr(f"[plan_pick_and_place] Failed to plan pick trajectory: {e}")
            rospy.logwarn(f"[DEBUG] Tried planning to pick pose {req.pick_pose} starting from joints {current_robot_joint_configuration}")
            return response  # Can't continue to place if pick fails

        # --- Placement Trajectory ---
        rospy.loginfo("[plan_pick_and_place] Planning placement trajectory...")
        log_pose(req.place_pose, label="Place Pose")

        move_group.clear_pose_targets()

        try:
            place_traj = plan_trajectory(move_group, req.place_pose, last_joint_positions)
            rospy.loginfo(f"[plan_pick_and_place] Placement trajectory points: {len(place_traj.joint_trajectory.points)}")
            response.trajectories.append(place_traj)

            rospy.loginfo(f"[DEBUG] Place trajectory first point: {place_traj.joint_trajectory.points[0].positions}")
            rospy.loginfo(f"[DEBUG] Place trajectory last point: {place_traj.joint_trajectory.points[-1].positions}")

        except Exception as e:
            rospy.logerr(f"[plan_pick_and_place] Failed to plan place trajectory: {e}")
            rospy.logwarn(f"[DEBUG] Tried planning to place pose {req.place_pose} starting from last pick joints: {last_joint_positions}")

    finally:
        try:
            move_group.clear_pose_targets()
        except Exception:
            pass

    rospy.loginfo(f"[plan_pick_and_place] Returning {len(response.trajectories)} trajectory(ies)")
    return response



def handle_mover_request(req):
    # wrapper so the Service can call plan_pick_and_place
    return plan_pick_and_place(req)


def collision_object_callback(msg: CollisionObject):
    """
    Add collision object primitives into the MoveIt planning scene.
    Each primitive in CollisionObject.primitives is added as a separate box item.
    """
    global planning_scene
    if planning_scene is None:
        rospy.logwarn("[collision_object_callback] planning_scene is None — ignoring collision object")
        return

    rospy.loginfo(f"[collision_object_callback] Received CollisionObject: id={msg.id}, frame={msg.header.frame_id}")

    for i, pose in enumerate(msg.primitive_poses):
        # Reconstruct PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header  # frame info from Unity
        pose_stamped.pose = pose

        # Give each box a unique name if multiple primitives exist
        box_name = f"{msg.id}_{i}"

        # Add box to MoveIt scene
        try:
            size_x, size_y, size_z = msg.primitives[i].dimensions
        except Exception as e:
            rospy.logerr(f"[collision_object_callback] Failed to read primitive dimensions: {e}")
            continue

        planning_scene.add_box(box_name, pose_stamped, size=(size_x, size_y, size_z))
        rospy.loginfo(f"[collision_object_callback] Requested add_box: {box_name} size=({size_x},{size_y},{size_z})")

        # Wait for the planning scene to register the object to avoid planning
        # races. If Scene doesn't get the object within timeout, we just continue.
        added = wait_for_scene_update(planning_scene, box_name, timeout=2.0)
        if added:
            rospy.loginfo(f"[collision_object_callback] Box added to planning scene: {box_name}")
        else:
            rospy.logwarn(f"[collision_object_callback] Timeout waiting for box to appear in scene: {box_name}")

    rospy.loginfo(f"[collision_object_callback] Processed CollisionObject: {msg.id}")


def main():
    global planning_scene

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('mover_service_node', anonymous=False)

    # Initialize PlanningSceneInterface once and reuse it
    planning_scene = PlanningSceneInterface()
    # small delay to let planning scene start
    rospy.sleep(0.5)

    service_name = rospy.get_param('~service_name', 'mover_service')
    rospy.loginfo(f"[mover_service_node] starting, service_name={service_name}, flip_y={rospy.get_param('~flip_y', False)}")

    rospy.Service(service_name, MoverService, handle_mover_request)
    rospy.loginfo(f"[mover_service_node] Service '{service_name}' ready to receive requests.")

    # Subscribe to Unity’s table/obstacle topic
    rospy.Subscriber("/collision_object", CollisionObject, collision_object_callback)
    rospy.loginfo("[mover_service_node] Subscribed to /collision_object for Unity obstacles")

    rospy.spin()


if __name__ == "__main__":
    main()

