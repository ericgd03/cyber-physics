#!/usr/bin/env python3

import rospy
import rosbag
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleRecorder:

    def __init__(self):

        rospy.init_node('turtle_recorder', anonymous=True)
        
        self.bag_file = '/home/student/catkin_ws/src/lyapunov/bags/trajectory.bag'
        self.bag = rosbag.Bag(self.bag_file, 'w')

        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        rospy.Subscriber('/turtle1/cmd_vel', Twist, self.cmd_vel_callback)

        self.rate = rospy.Rate(10)

    def pose_callback(self, msg):

        rospy.loginfo("Recording pose: %s", msg)
        self.bag.write('/turtle1/pose', msg)

    def cmd_vel_callback(self, msg):

        rospy.loginfo("Recording cmd_vel: %s", msg)
        self.bag.write('/turtle1/cmd_vel', msg)

    def record(self):

        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.bag.close()

if __name__ == '__main__':
    recorder = TurtleRecorder()
    recorder.record()