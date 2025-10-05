#!/usr/bin/env python3
import rospy
from ur10e_rg2_moveit.srv import PoseEstimationService, PoseEstimationServiceResponse
from geometry_msgs.msg import Pose, Point, Quaternion

def handle_pose_estimation(req):
    rospy.loginfo("Received image data of length: %d", len(req.image.data))
    
    # Return a dummy pose (position + orientation)
    response = PoseEstimationServiceResponse()
    pose = Pose()
    pose.position  = Point(0.5,0.0,0.5)
    pose.orientation = Quaternion(0,0,0,1)
    response.estimated_pose = pose
    return response

def pose_estimation_service():
    rospy.init_node('dummy_pose_estimation_service')
    s = rospy.Service('pose_estimation_srv', PoseEstimationService, handle_pose_estimation)
    rospy.loginfo("Dummy Pose Estimation Service ready.")
    rospy.spin()

if __name__ == "__main__":
    pose_estimation_service()
