#!/usr/bin/env python
import rospy, sys, moveit_commander
from ur10e_rg2_moveit.srv import MoverService, MoverServiceResponse
from sim_real_pnp import plan_pick_and_place  # your custom planner

def handle_mover_service(req):
    rospy.loginfo("📩 Received pick/place request from Unity.")
    rospy.loginfo(f"Pick pose: {req.pick_pose.position}")
    rospy.loginfo(f"Place pose: {req.place_pose.position}")

    try:
        response = plan_pick_and_place(req)  # Should return MoverServiceResponse
        rospy.loginfo("✅ Motion plan computed.")
        return response
    except Exception as e:
        rospy.logerr(f"❌ Planning failed: {e}")
        return MoverServiceResponse()  # Empty response if failed

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("mover_service_node", anonymous=True)
    rospy.Service("mover_service", MoverService, handle_mover_service)
    rospy.loginfo("🟢 MoverService ready for Unity.")
    rospy.spin()

