#!/usr/bin/env python

from __future__ import print_function

import rospy

import sys
import copy
import math
import moveit_commander
import tf

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from ur10e_rg2_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['robot_shoulder_pan_joint', 'robot_shoulder_lift_joint', 'robot_elbow_joint', 'robot_wrist_1_joint', 'robot_wrist_2_joint', 'robot_wrist_3_joint']

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan
        
"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles, 
                    max_retries=3, initial_timeout=20.0, timeout_increment=5.0):
    """
    Plan a trajectory to a given destination_pose starting from start_joint_angles,
    with retries and dynamic timeout.

    Args:
        move_group (MoveGroupCommander): The MoveGroupCommander instance for planning.
        destination_pose (geometry_msgs.msg.Pose): The target pose.
        start_joint_angles (list): Starting joint angles.
        max_retries (int): Maximum number of retries for planning.
        initial_timeout (float): Initial planning timeout in seconds.
        timeout_increment (float): Increment in timeout for each retry.

    Returns:
        moveit_commander.RobotTrajectory: The planned trajectory.
    
    Raises:
        Exception: If planning fails after all retries.
    """
    # Initialize the current joint state
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)
    destination_pose.position.y = -destination_pose.position.y
    move_group.set_pose_target(destination_pose)

    # Retry mechanism with dynamic timeout
    timeout_seconds = initial_timeout
    for attempt in range(max_retries):
        rospy.loginfo(f"Attempt {attempt + 1} of {max_retries} to plan trajectory.")
        rospy.loginfo(f"Setting planning timeout to {timeout_seconds} seconds.")
        
        move_group.set_planning_time(timeout_seconds)
        plan = move_group.plan()

        if plan:  # Check if a valid plan was generated
            planned_trajectory = planCompat(plan)  # Process the plan using planCompat
            if hasattr(planned_trajectory, 'joint_trajectory') and planned_trajectory.joint_trajectory.points:
                rospy.loginfo("Trajectory planning succeeded.")
                return planned_trajectory  # Return the valid planned trajectory

        rospy.logwarn(f"Attempt {attempt + 1} failed. Increasing timeout and retrying...")
        timeout_seconds += timeout_increment  # Increment timeout for the next attempt

    # Raise an exception if all retries fail
    exception_str = (
        f"Trajectory could not be planned after {max_retries} attempts.\n"
        f"Destination pose: {destination_pose}, starting joint angles: {start_joint_angles}.\n"
        "Please make sure the target and destination are reachable by the robot."
    )
    rospy.logerr(exception_str)
    raise Exception(exception_str)

# def plan_trajectory(move_group, destination_pose, start_joint_angles): 
#     current_joint_state = JointState()
#     current_joint_state.name = joint_names
#     current_joint_state.position = start_joint_angles

#     moveit_robot_state = RobotState()
#     moveit_robot_state.joint_state = current_joint_state
#     move_group.set_start_state(moveit_robot_state)

#     move_group.set_pose_target(destination_pose)
    
#     # Set planning time limit
#     move_group.set_planning_time(10.0)
    
#     plan = move_group.plan()

#     if not plan:
#         exception_str = """
#             Trajectory could not be planned for a destination of {} with starting joint angles {}.
#             Please make sure target and destination are reachable by the robot.
#         """.format(destination_pose, destination_pose)
#         raise Exception(exception_str)

#     return planCompat(plan)


"""
    Creates a pick and place plan using the four states below.
    
    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position

    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved

    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""
def log_pose(pose, label="Pose"):
    # ROS position
    ros_position = pose.position
    rospy.loginfo(f"{label} - ROS Position: x={ros_position.x}, y={ros_position.y}, z={ros_position.z}")

    # # ROS to Unity coordinate transformation (swap X and Z, negate X)
    # unity_position = geometry_msgs.msg.Point()
    # unity_position.x = -pose.position.y
    # unity_position.y = pose.position.z
    # unity_position.z = pose.position.x 

    # # Log the position in Unity coordinates (in meters)
    # rospy.loginfo(f"{label} - Unity Position: x={unity_position.x}, y={unity_position.y}, z={unity_position.z}")

    # # Convert quaternion orientation to Euler angles (roll, pitch, yaw) in degrees
    # orientation = pose.orientation
    # euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    # roll_deg, pitch_deg, yaw_deg = map(math.degrees, euler)  # Convert radians to degrees

    # rospy.loginfo(f"{label} - Orientation (Euler Angles in Degrees): roll={roll_deg}, pitch={pitch_deg}, yaw={yaw_deg}")
    
def plan_pick_and_place(req):
    response = MoverServiceResponse()

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    current_robot_joint_configuration = req.joints_input.joints
    rospy.loginfo(f"Current Robot Joint Configuration: {current_robot_joint_configuration}")
    
    current_pose = move_group.get_current_pose()
    rospy.loginfo("Current pose: {current_pose}")
    
    def log_and_fail(stage, error_message):
        rospy.logerr(f"Failed to plan {stage} trajectory: {error_message}")
        return response

    try:
        # Pre-Grasp - position gripper above the target   
        # current_pose.pose.position.x = current_pose.pose.position.x - 0.1
        # current_pose.pose.position.y = current_pose.pose.position.y - 0.1
        # current_pose.pose.position.z = current_pose.pose.position.z - 0.1
        rospy.loginfo("Pre-Grasp Pose:")
        log_pose(req.pick_pose, label="Pre-Grasp Pose")
        
        current_joint_values = move_group.get_current_joint_values()
        rospy.loginfo(f"Current Joint Values: {current_joint_values}")
        pre_grasp_pose = plan_trajectory(move_group, 
                                        # current_pose
                                        req.pick_pose
                                         , 
                                        #  current_joint_values
                                        current_robot_joint_configuration
                                         )
        # previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

        # # Grasp - lower gripper to grasp the object
        # pick_pose = copy.deepcopy(req.pick_pose)
        # pick_pose.position.z -= 0.05  # Static value; consider passing dynamically
        
        # rospy.loginfo("Grasp Pose:")
        # log_pose(pick_pose, label="Grasp Pose")
        
        # grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)
        # previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

        # # Pick Up - raise gripper back to the pre-grasp position
        # rospy.loginfo("Pick-Up Pose:")
        # log_pose(req.pick_pose, label="Pick-Up Pose")
        
        # pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)
        # previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

        # # Place - move gripper to the desired placement position
        # rospy.loginfo("Place Pose:")
        # log_pose(req.place_pose, label="Place Pose")
        # place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)

        # Append all successful trajectories to the response
        response.trajectories.append(pre_grasp_pose)
        # response.trajectories.append(grasp_pose)
        # response.trajectories.append(pick_up_pose)
        # response.trajectories.append(place_pose)

    except Exception as e:
        # Handle errors from `plan_trajectory` or other stages
        return log_and_fail("unknown", str(e))

    finally:
        # Clear targets to ensure no lingering state in the MoveGroupCommander
        move_group.clear_pose_targets()

    rospy.loginfo(f"response: {response.trajectories}")
    return response

# def plan_pick_and_place(req):
#     response = MoverServiceResponse()

#     group_name = "arm"
#     move_group = moveit_commander.MoveGroupCommander(group_name)

#     current_robot_joint_configuration = req.joints_input.joints

#     # Pre grasp - position gripper directly above target object
#     pre_grasp_pose = plan_trajectory(move_group, req.pick_pose, current_robot_joint_configuration)
    
#     # If the trajectory has no points, planning has failed and we return an empty response
#     if not pre_grasp_pose.joint_trajectory.points:
#         rospy.logerr("Failed to plan pre-grasp trajectory")
#         response.error_message = "Pre-grasp trajectory planning failed."
#         return response

#     previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

#     # Grasp - lower gripper so that fingers are on either side of object
#     pick_pose = copy.deepcopy(req.pick_pose)
#     pick_pose.position.z -= 0.05  # Static value coming from Unity, TODO: pass along with request
#     grasp_pose = plan_trajectory(move_group, pick_pose, previous_ending_joint_angles)
    
#     if not grasp_pose.joint_trajectory.points:
#         return response

#     previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

#     # Pick Up - raise gripper back to the pre grasp position
#     pick_up_pose = plan_trajectory(move_group, req.pick_pose, previous_ending_joint_angles)
    
#     if not pick_up_pose.joint_trajectory.points:
#         return response

#     previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

#     # Place - move gripper to desired placement position
#     place_pose = plan_trajectory(move_group, req.place_pose, previous_ending_joint_angles)

#     if not place_pose.joint_trajectory.points:
#         return response

#     # If trajectory planning worked for all pick and place stages, add plan to response
#     response.trajectories.append(pre_grasp_pose)
#     response.trajectories.append(grasp_pose)
#     response.trajectories.append(pick_up_pose)
#     response.trajectories.append(place_pose)

#     move_group.clear_pose_targets()

#     return response


def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur10e_rg2_moveit_server')

    s = rospy.Service('ur10e_rg2_moveit', MoverService, plan_pick_and_place)
    # plan_pick_and_place()
    print("Ready to plan")
    rospy.spin()

if __name__ == "__main__":
    moveit_server()