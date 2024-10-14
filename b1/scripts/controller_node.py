#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose2D
import math

class controller_node:

    def __init__(self):
        
        rospy.init_node("controller_node", anonymous=True)
        self.velocity_pub = rospy.Publisher("/smoother_cmd_vel", Twist, queue_size=10)
        self.odometry_sub = rospy.Subscriber("/odometry_b1", Pose2D, self.odometry_callback)
        self.waypoint_sub = rospy.Subscriber("/waypoint", Pose2D, self.waypoints_callback)
        self.imu_sub = rospy.Subscriber("/Gz", Float32, self.imu_callback)
        #Suscriber to ultrasonic --> To avoid obstacles
        #Suscriber to xarm --> Conformation to move gain
        #Suscriber to omnron --> Conformation to start rutine
        self.rate = rospy.Rate(10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0

        self.error_distance = 0.0
        self.error_angle = 0.0

        self.linear_vel = 0.0
        self.angular_vel = 0.0

        #self.kp_linear = 1
        self.kp_angular = 0.2

        self.trajectory_state = ''
        self.turn_counter = 0


    def odometry_callback(self, msg):
        
        self.x = msg.x
        self.y = msg.y

    def waypoints_callback(self, msg):
        
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.theta_goal = msg.theta

    def imu_callback(self, msg):

        self.theta = msg.data

    def control_law(self):
        
        distance_error = math.sqrt((self.x_goal - self.x)**2 + (self.y_goal - self.y)**2)
        angle_error = math.atan2(self.y_goal - self.y, self.x_goal - self.x)

        #if distance_error < 0.1:
        #    self.linear_vel = 0.0
        #    self.angular_vel = 0.0
        #else:
            #linear_vel = self.kp_linear * distance_error
        #    self.linear_vel = 0.2
        #    self.angular_vel = self.kp_angular * self.normalize_angle(self.theta - angle_error)

        if (self.trajectory_state == ''):

            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.trajectory_state = 'MOVE_X'

        elif (self.trajectory_state == 'MOVE_X'):

            x_error = (self.x_goal - self.x)
            print("Error X:", x_error)

            if (x_error <= 0.05):
                self.linear_vel = 0.0
                current_theta = self.theta
                self.trajectory_state = 'TURN_TO_Y'
            else:
                self.linear_vel = 0.3

        elif (self.trajectory_state == 'TURN_TO_Y'):

            orientation = self.y_goal - self.y
            angle_change = self.theta - current_theta
            rospy.loginfo("Angle error: ", angle_change)

            if (orientation > 0):
                self.angular_vel = 0.5
                if (angle_change >= 90.0):
                    self.angular_vel = 0.0
                    self.trajectory_state = 'MOVE_Y'
            if (orientation < 0):
                self.angular_vel = -0.5
                if (angle_change <= -90.0):
                    self.angular_vel = 0.0
                    self.trajectory_state = 'MOVE_Y'

        elif (self.trajectory_state == 'MOVE_Y'):

            y_error = self.y_goal - self.y
            print("Error Y:", y_error)

            if (y_error <= 0.05):
                self.linear_vel = 0.0
                self.trajectory_state = 'TURN_TO_X'
            else:
                self.linear_vel = 0.3

        elif (self.trajectory_state == 'TURN_TO_X'):

            rospy.loginfo("turn_to_x")

            orientation = self.y_goal - self.y

            if ((orientation) > 0):
                self.angular_vel = -0.5
            elif (orientation < 0):
                self.angular_vel = 0.5
            else:
                self.trajectory_state = ''

            self.turn_counter += 1
            if self.turn_counter == 35:

                self.angular_vel = 0.0
                self.turn_counter = 0
                self.trajectory_state = ''
                # Bandera de xarm que no va a mover el b1

        velocity = Twist()
        velocity.linear.x = self.linear_vel
        velocity.angular.z = self.angular_vel
        self.velocity_pub.publish(velocity)

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def move_robot(self):

        while not rospy.is_shutdown():
            self.control_law()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        c_n = controller_node()
        c_n.move_robot()
    except rospy.ROSInterruptException:
        pass