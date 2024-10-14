#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
import math

class waypoints_node:

    def __init__(self):
        
        rospy.init_node("waypoints_node", anonymous=True)
        self.waypoints_pub = rospy.Publisher("/waypoint", Pose2D, queue_size=10)
        self.odometry_sub = rospy.Subscriber('/odometry_b1', Pose2D, self.odom_callback)
        rospy.Rate(10)

        self.waypoints = [
            # X: 1m = 0.3 -- Y: 1m = 0.1
            #Pose2D(1.0, 0.1, 0.0)

            Pose2D(0.2, 0.1, 0.0),
            Pose2D(0.2, 0.2, 0.0)

            #Pose2D(0.3, 0.3, 0.0)
            #Pose2D(6.0, -3.0, 0.0)
            #Pose2D(0.0, 0.0, 0.0)          
        ]
        self.current_waypoint_index = 0
        self.threshold_distance = 0.05

    def odom_callback(self, msg):

        x = msg.x
        y = msg.y

        waypoint = self.waypoints[self.current_waypoint_index]
        self.waypoints_pub.publish(waypoint)

        distance_to_waypoint = math.sqrt((waypoint.x - x)**2 + (waypoint.y - y)**2)

        if distance_to_waypoint < self.threshold_distance:

            self.current_waypoint_index += 1

            if self.current_waypoint_index >= len(self.waypoints):

                #self.current_waypoint_index = 0
                rospy.signal_shutdown("Trajectory completed")
                return

if __name__ == "__main__":
    try:
        w_n = waypoints_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass