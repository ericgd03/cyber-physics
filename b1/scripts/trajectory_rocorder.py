#!/usr/bin/env python3

import rospy
import rosbag
from geometry_msgs.msg import Twist, Pose2D

class trajectory_recorder:

    def __init__(self):

        rospy.init_node('trajectory_recorder', anonymous=True)
        #rospy.Subscriber('/odometry_b1', Pose2D, self.pose_callback)
        rospy.Subscriber('/smoother_cmd_vel', Twist, self.cmd_vel_callback)
        self.rate = rospy.Rate(10)
        
        self.bag_file = '/home/student/catkin_ws/src/b1/bags/trajectory.bag'
        self.bag = rosbag.Bag(self.bag_file, 'w')

    #def pose_callback(self, msg):
        #rospy.loginfo("Recording pose: %s", msg)
        #self.bag.write('/turtle1/pose', msg)

    def cmd_vel_callback(self, msg):

        self.bag.write('/smoother_cmd_vel', msg)

    def record(self):

        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.bag.close()

if __name__ == '__main__':
    recorder = trajectory_recorder()
    recorder.record()