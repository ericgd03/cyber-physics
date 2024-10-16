#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int16, Bool, String
from geometry_msgs.msg import Twist, Pose2D
import math

class controller_node:

    def __init__(self):
        
        rospy.init_node("controller_node", anonymous=True)

        self.velocity_pub = rospy.Publisher("/smoother_cmd_vel", Twist, queue_size=10)
        self.odometry_sub = rospy.Subscriber("/odometry_b1", Pose2D, self.odometry_callback)
        self.waypoint_sub = rospy.Subscriber("/waypoint", Pose2D, self.waypoints_callback)

        self.imu_sub = rospy.Subscriber("/Gz", Float32, self.imu_callback)

        self.xarm_pub = rospy.Publisher("/xarm_routine", Int16, queue_size=10)
        self.xarm_sub = rospy.Subscriber("/xarm_movement", Bool, self.xarm_callback)

        self.reset_origin_pub = rospy.Publisher("/reset_origin", Bool, queue_size=10)
        self.done_trajectory_sub = rospy.Subscriber("/done_trajectory", Bool, self.done_trajectory_callback)

        #Suscriber to ultrasonic --> To avoid obstacles
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

        self.stop_vel = 0.0
        self.forward_vel = 0.25
        self.turn_vel = 0.3
        self.turn_degrees = 88
        self.threshold_distance = 0.05

        #self.kp_linear = 1
        self.kp_angular = 0.2

        self.current_theta = 0
        self.trajectory_state = ''
        self.done_trajectory = False

        self.done_moving_xarm = False
        self.xarm_routine = 1

    def odometry_callback(self, msg):
        
        self.x = msg.x
        self.y = msg.y

    def waypoints_callback(self, msg):
        
        self.x_goal = msg.x
        self.y_goal = msg.y
        self.theta_goal = msg.theta

    def imu_callback(self, msg):

        self.theta = msg.data

    def xarm_callback(self, msg):

        self.done_moving_xarm = msg.data

    def done_trajectory_callback(self, msg):

        self.done_trajectory = msg.data
        print("Stopping the robot, trajectory finished")

    def control_law(self):
        
        #distance_error = math.sqrt((self.x_goal - self.x)**2 + (self.y_goal - self.y)**2)
        #angle_error = math.atan2(self.y_goal - self.y, self.x_goal - self.x)

        #if distance_error < 0.1:
        #    self.linear_vel = 0.0
        #    self.angular_vel = 0.0
        #else:
            #linear_vel = self.kp_linear * distance_error
        #    self.linear_vel = 0.2
        #    self.angular_vel = self.kp_angular * self.normalize_angle(self.theta - angle_error)

        if (self.trajectory_state == ''):

            self.linear_vel = self.stop_vel
            self.angular_vel = self.stop_vel

            if (self.xarm_routine > 3):
                self.xarm_routine = 1

            if (self.done_trajectory == False):
                self.trajectory_state = 'MOVE_X'
            else:
                pass#self.trajectory_state = "RETURN_TO_ORIGIN"

        elif (self.trajectory_state == 'MOVE_X'):

            x_error = (self.x_goal - self.x)
            print("Error X:", x_error)

            if (x_error <= self.threshold_distance):
                self.linear_vel = self.stop_vel
                self.current_theta = self.theta
                self.trajectory_state = 'TURN_TO_Y'
            else:
                self.linear_vel = self.forward_vel

        elif (self.trajectory_state == 'TURN_TO_Y'):

            orientation = self.y_goal - self.y
            angle_change = self.theta - self.current_theta
            angle = int(angle_change)
            print("Angle error: ", angle)

            if (orientation == 0):
                self.angular_vel = self.stop_vel
                self.trajectory_state = 'MOVE_Y'
            elif (orientation > 0):
                self.angular_vel = self.turn_vel
                # Menos 5 para compensar que gira un poco mas
                if (angle_change >= self.turn_degrees):
                    self.angular_vel = self.stop_vel
                    self.trajectory_state = 'MOVE_Y'
            elif (orientation < 0):
                self.angular_vel = -self.turn_vel
                if (angle_change <= -self.turn_degrees):
                    self.angular_vel = self.stop_vel
                    self.trajectory_state = 'MOVE_Y'

        elif (self.trajectory_state == 'MOVE_Y'):

            y_error = self.y_goal - self.y
            print("Error Y:", y_error)

            if (y_error <= self.threshold_distance):
                self.linear_vel = self.stop_vel
                #self.current_theta = self.theta
                self.reset_origin_pub.publish(True)
                if (self.xarm_routine >= 3):
                    self.trajectory_state = ""
                else:
                    self.xarm_pub.publish(self.xarm_routine)
                    self.trajectory_state = 'MOVE_XARM'
                #self.trajectory_state = 'TURN_TO_X'
            else:
                self.linear_vel = self.forward_vel

        #elif (self.trajectory_state == 'TURN_TO_X'):

            #orientation = self.y_goal - self.y
            #angle_change = self.theta - self.current_theta
            #angle = int(angle_change)
            #print("Angle error: ", angle)

            #if (orientation < 0):
                #self.angular_vel = self.turn_vel
                #if (angle_change >= self.turn_degrees):
                    #self.angular_vel = self.stop_vel
                    #self.reset_origin_pub.publish(True)
                    #self.xarm_pub.publish(self.xarm_routine)
                    #self.trajectory_state = ''
            #if (orientation > 0):
                #self.angular_vel = -self.turn_vel
                #if (angle_change <= -self.turn_degrees):
                    #self.angular_vel = self.stop_vel
                    #self.reset_origin_pub.publish(True)
                    #self.xarm_pub.publish(self.xarm_routine)
                    #self.trajectory_state = ''

            #orientation = self.y_goal - self.y
            #if ((orientation) > 0):
            #    self.angular_vel = -0.5
            #elif (orientation < 0):
            #    self.angular_vel = 0.5
            #else:
            #    self.trajectory_state = ''
            #self.turn_counter += 1
            #if self.turn_counter == 35:
            #    self.angular_vel = 0.0
            #    self.turn_counter = 0
            #    self.trajectory_state = ''
            #    # Bandera de xarm que no va a mover el b1

        elif (self.trajectory_state == "MOVE_XARM"):

            print("Moving xArm, current routine: ", self.xarm_routine)

            self.linear_vel = self.stop_vel
            self.angular_vel = self.stop_vel

            if (self.done_moving_xarm == True):
                self.done_moving_xarm = False
                self.xarm_routine += 1
                self.trajectory_state = ""

        elif (self.trajectory_state == "RETURN_TO_ORIGIN"):

            pass
            #play rosbag
            #self.done_trajectory = False
            #self.trajectory_state = ""

        else:
            print("ERROR - PLEASE RESTART THE SYSTEM")
            self.linear_vel = self.stop_vel
            self.angular_vel = self.stop_vel

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