#!/usr/bin/env python
from __future__ import print_function

import rospy
import sys
import copy
import math
import moveit_commander
import tf

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list

from ur10e_rg2_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = [
    'robot_shoulder_pan_joint', 'robot_shoulder_lift_joint', 'robot_elbow_joint',
    'robot_wrist_1_joint', 'robot_wrist_2_joint', 'robot_wrist_3_joint'
]

# compatibility helper for MoveIt plan() return type
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan


def plan_trajectory(move_group, destination_pose, start_joint_angles,
                    max_retries=3, initial_timeout=20.0, timeout_increment=5.0):
    # Safe copy so we don't mutate the incoming request
    dest = copy.deepcopy(destination_pose)

    # Build a start state from given joint angles
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    # Optional coordinate flip controlled by ROS param (~flip_y)
    flip_y = rospy.get_param('~flip_y', False)
    if flip_y:
        dest.position.y = -dest.position.y

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
    response = MoverServiceResponse()
    
    rospy.loginfo("[plan_pick_and_place] Received a request!")

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    rospy.loginfo(f"[plan_pick_and_place] MoveGroupCommander created for group: {group_name}")

    current_robot_joint_configuration = req.joints_input.joints
    rospy.loginfo(f"[plan_pick_and_place] Current joint configuration: {current_robot_joint_configuration}")

    try:
        rospy.loginfo("[plan_pick_and_place] Attempting to plan pick trajectory...")
        log_pose(req.pick_pose, label="Pick Pose")
        
        rospy.loginfo("[plan_pick_and_place] About to plan pre-grasp trajectory...")

        pre_grasp_traj = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
        rospy.loginfo(f"[plan_pick_and_place] pre-grasp trajectory planned: {pre_grasp_traj}")
        rospy.loginfo(f"[plan_pick_and_place] Got trajectory with {len(pre_grasp_traj.joint_trajectory.points)} points")

        response.trajectories.append(pre_grasp_traj)
        rospy.loginfo(f"[plan_pick_and_place] trajectories in response: {len(response.trajectories)}")
        for i, traj in enumerate(response.trajectories):
    	    rospy.loginfo(f"Trajectory {i}: {traj.joint_trajectory.points}")

    except Exception as e:
        rospy.logerr(f"[plan_pick_and_place] Planning failed: {e}")
        return response

    finally:
        move_group.clear_pose_targets()

    rospy.loginfo(f"[plan_pick_and_place] Returning {len(response.trajectories)} trajectory(ies)")
    return response



def handle_mover_request(req):
    # wrapper so the Service can call plan_pick_and_place
    return plan_pick_and_place(req)


def main():
    # initialize ROS + MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('mover_service_node', anonymous=False)

    service_name = rospy.get_param('~service_name', 'mover_service')
    rospy.loginfo(f"[mover_service_node] starting, service_name={service_name}, flip_y={rospy.get_param('~flip_y', False)}")

    rospy.Service(service_name, MoverService, handle_mover_request)
    rospy.loginfo(f"[mover_service_node] Service '{service_name}' ready to receive requests.")
    rospy.spin()


if __name__ == "__main__":
    main()

