#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from turtlesim.msg import Pose
from std_msgs.msg import Header
import math

class PathPublisherNode:

    def __init__(self):

        rospy.init_node('path_publisher_node', anonymous=True)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.path_publisher = rospy.Publisher('/turtle1/path', Path, queue_size=10)
        
        self.path = Path()
        self.path.header = Header(frame_id="world")

    def pose_callback(self, data):

        pose_stamped = PoseStamped()
        pose_stamped.header = Header(stamp=rospy.Time.now(), frame_id="world")
        pose_stamped.pose.position.x = data.x
        pose_stamped.pose.position.y = data.y
        pose_stamped.pose.position.z = 0

        # Adding orientation (for RViz), assuming the turtle's z-axis corresponds to theta (2D plane)
        pose_stamped.pose.orientation.z = math.sin(data.theta / 2.0)
        pose_stamped.pose.orientation.w = math.cos(data.theta / 2.0)

        # Append the current pose to the path
        self.path.poses.append(pose_stamped)

        # Publish the path
        self.path.header.stamp = rospy.Time.now()
        self.path_publisher.publish(self.path)

if __name__ == '__main__':
    try:
        path_node = PathPublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
