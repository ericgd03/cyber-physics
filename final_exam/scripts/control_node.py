#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose2D
import math
from turtlesim.msg import Pose

class control_node:
    
    def __init__(self):
    
        rospy.init_node('lyapunov_controller', anonymous=True)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.waypoint_subscriber = rospy.Subscriber('/point', Pose2D, self.waypoint_callback)
        self.rate = rospy.Rate(10)
        
        self.x_goal = 0.0
        self.y_goal = 0.0

        self.x = 0
        self.y = 0
        self.theta = 0

        #self.k_linear = 1.0
        self.k_angular = 4.0

    def update_pose(self, data):

        self.x = data.x
        self.y = data.y
        self.theta = data.theta

    def waypoint_callback(self, msg):

        self.x_goal = msg.x
        self.y_goal = msg.y

    def control_law(self):

        distance = math.sqrt((self.x_goal - self.x)**2 + (self.y_goal - self.y)**2)
        angle_to_goal = math.atan2(self.y_goal - self.y, self.x_goal - self.x)

        if distance < 0.2:
            linear_velocity = 0.0
            angular_velocity = 0.0
        else:
            # linear_velocity = self.k_linear * distance
            linear_velocity = 0.8
            angular_velocity = self.k_angular * self.normalize_angle(angle_to_goal - self.theta)

        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity
        vel_msg.angular.z = angular_velocity
        self.velocity_publisher.publish(vel_msg)

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def move_turtle(self):

        while not rospy.is_shutdown():
            self.control_law()
            self.rate.sleep()

if __name__ == '__main__':
    
    try:
        controller = control_node()
        controller.move_turtle()
    except rospy.ROSInterruptException:
        pass
