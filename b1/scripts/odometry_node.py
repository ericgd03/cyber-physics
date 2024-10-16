#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Pose2D
import math

TICKS_PER_REV = 664
WHEEL_DIAMETER = 0.10
WHEEL_BASE = 0.25 

class odometry_node:

    def __init__(self):
        
        rospy.init_node("odometry_node", anonymous=True)
        self.odometry_pub = rospy.Publisher("/odometry_b1", Pose2D, queue_size=10)
        self.left_encoder_sub = rospy.Subscriber("/Lvel", Int16, self.left_encoder_callback)
        self.right_encoder_sub = rospy.Subscriber("/Rvel", Int16, self.right_encoder_callback)
        self.reset_origin_sub = rospy.Subscriber("/reset_origin", Bool, self.reset_origin_callback)
        self.rate = rospy.Rate(10)

        self.rpm_left = 0.0
        self.rpm_right = 0.0

        #self.left_ticks = 0.0
        #self.right_ticks = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = rospy.Time.now()

    def left_encoder_callback(self, msg):

        #self.left_ticks = (msg.data - 25)
        self.rpm_left = msg.data

    def right_encoder_callback(self, msg):

        #self.right_ticks = msg.data
        self.rpm_right = msg.data

    def reset_origin_callback(self, msg):
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def odometry(self):
        
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        w_left = self.rpm_left * (2 * math.pi / 60)
        w_right = self.rpm_right * (2 * math.pi / 60)

        #w_left = 2 * math.pi * self.left_ticks / TICKS_PER_REV * dt
        #w_right = 2 * math.pi * self.right_ticks / TICKS_PER_REV * dt

        v_left = w_left * WHEEL_DIAMETER / 2
        v_right = w_right * WHEEL_DIAMETER / 2

        v = (v_right + v_left) / 2
        w = (v_right - v_left) / WHEEL_BASE

        self.theta += w * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        odom = Pose2D()
        odom.x = self.x
        odom.y = self.y
        odom.theta = self.theta
        self.odometry_pub.publish(odom)

    def start_odometry(self):

        while not rospy.is_shutdown():
            self.odometry()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        o_n = odometry_node()
        o_n.start_odometry()
    except rospy.ROSInterruptException:
        pass