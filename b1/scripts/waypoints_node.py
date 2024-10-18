#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose2D
import math

class waypoints_node:

    def __init__(self):
        
        rospy.init_node("waypoints_node", anonymous=True)
        self.waypoints_pub = rospy.Publisher("/waypoint", Pose2D, queue_size=10)
        #self.odometry_sub = rospy.Subscriber('/odometry_b1', Pose2D, self.odom_callback)
        self.reset_origin_sub = rospy.Subscriber("/reset_origin", Bool, self.reset_origin_callback)
        self.done_trajectory_pub = rospy.Publisher("/done_trajectory", Bool, queue_size=10)
        self.rate = rospy.Rate(10)

        self.waypoints = [
            # X: 1m = 0.3 -- Y: 1m = 0.1
            #1.02
            Pose2D(0.945, 0.084, 0.0),
            Pose2D(0.37, 0.0, 0.0),
            Pose2D(0.0, 0.40, 0.0),
            Pose2D(0.0, 0.22, 0.0),
            Pose2D(0.01, 0.01, 0.0),

            #Pose2D(1.0, 1.0, 0.0),
            #Pose2D(1.0, 1.5, 0.0),
            #Pose2D(0.0, 1.0, 0.0),
        ]
        self.current_waypoint_index = 0
        self.threshold_distance = 0.01
        self.next_waypoint = False
    
    def reset_origin_callback(self, msg):

        self.current_waypoint_index += 1

    def send_waypoint(self):

        if self.current_waypoint_index >= len(self.waypoints):

            self.current_waypoint_index = 0
            self.done_trajectory_pub.publish(True)
            #rospy.signal_shutdown("Trajectory completed")
            #return

        waypoint = self.waypoints[self.current_waypoint_index]
        self.waypoints_pub.publish(waypoint)

if __name__ == "__main__":
    try:
        w_n = waypoints_node()
        while not rospy.is_shutdown():
            w_n.send_waypoint()
            w_n.rate.sleep()
    except rospy.ROSInterruptException:
        pass