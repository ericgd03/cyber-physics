#!/usr/bin/env python3

import rospy
from final_exam.srv import SelectTrajectory, SelectTrajectoryResponse

def handle_select_trajectory(req):

    if req.trajectory_number in [1, 2, 3]:
        message = f"Trajectory {req.trajectory_number} selected."
        rospy.set_param('/selected_trajectory', req.trajectory_number)
        rospy.loginfo(message)
        return SelectTrajectoryResponse(success=True, message=message)
    else:
        rospy.logwarn("Invalid trajectory number, please select 1, 2, or 3.")
        return SelectTrajectoryResponse(success=False, message="Invalid number. Please select 1, 2, or 3.")

def select_trajectory_server():
    
    rospy.init_node('trajectory_service_node')
    rospy.Service('/select_trajectory', SelectTrajectory, handle_select_trajectory)
    rospy.loginfo("Ready to select trajectory (1, 2, or 3).")
    rospy.spin()

if __name__ == "__main__":
    select_trajectory_server()
