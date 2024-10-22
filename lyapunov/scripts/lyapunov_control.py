#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

current_x = 0.0
current_y = 0.0
current_theta = 0.0

desired_x = 0.0
desired_y = 0.0
desired_theta = 0.0

k_linear = 0.1
k_angular = 0.1

def pose_callback(data):

    global current_x, current_y, current_theta
    
    current_x = data.x
    current_y = data.y
    current_theta = data.theta

def lyapunov():

    global desired_x, desired_y, desired_theta

    rospy.init_node('lyapunov', anonymous=True)
    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
    rate = rospy.Rate(100)
    vel_msg = Twist()

    desired_x = float(input("Desired x: "))
    desired_y = float(input("Desired y: "))
    desired_theta = float(input("Desired theta: "))

    while not rospy.is_shutdown():
         
        rho = math.sqrt((desired_x - current_x)**2 + (desired_y - current_y)**2)
        alpha = math.atan2(desired_y - current_y, desired_x - current_x)

        vel_msg.linear.x = k_linear * rho
        vel_msg.angular.z = k_angular * (alpha - current_theta)

        velocity_publisher.publish(vel_msg)

        rospy.Rate.sleep()

if __name__ == "__main__":
    try:
        lyapunov()
    except rospy.ROSInterruptException:
        pass