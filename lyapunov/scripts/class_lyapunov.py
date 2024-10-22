#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Pose2D
import math
from turtlesim.srv import Spawn, SpawnResponse

class lyapunov_controller:
    
    def __init__(self):
        
        #self.x_goal = x_desired
        #self.y_goal = y_desired
        self.x_goal = 0.0
        self.y_goal = 0.0

        self.x = 0
        self.y = 0
        self.theta = 0

        rospy.init_node('lyapunov_controller', anonymous=True)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.waypoint_subscriber = rospy.Subscriber('/point', Pose2D, self.waypoint_callback)
        service = rospy.Service('/spawn', Spawn, self.spawn_callback)
        self.rate = rospy.Rate(10)

        # self.k_linear = 1.0
        self.k_angular = 4.0

        #self.spawn_called = False
        self.turtle_counter = 1

        self.last_time = rospy.Time.now()

        self.trajectory_state = ''
        self.turn_counter = 0

    def update_pose(self, data):

        self.x = data.x
        self.y = data.y
        self.theta = data.theta

    def waypoint_callback(self, msg):

        self.x_goal = msg.x
        self.y_goal = msg.y

    def spawn_callback(self, request):
        
        self.turtle_counter += 1
        #self.spawn_called = True
        return SpawnResponse(request.x, request.y, request.theta, request.name)

    def control_law(self):

        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        #rospy.loginfo("Current turtles: %i", self.turtle_counter)
        
        if self.turtle_counter >= 3:
            rospy.loginfo("STOPPING. Currently there are %i turtles.", self.turtle_counter)
            linear_velocity = 0.0
            angular_velocity = 0.0
        else:
            distance = math.sqrt((self.x_goal - self.x)**2 + (self.y_goal - self.y)**2)
            angle_to_goal = math.atan2(self.y_goal - self.y, self.x_goal - self.x)
            #print("angle to goal ", angle_to_goal)
            #print("theta ", self.theta)

            #if distance < 0.2:
            #    linear_velocity = 0.0
            #    angular_velocity = 0.0
            #else:
                # linear_velocity = self.k_linear * distance
            #    linear_velocity = 1
            #    angular_velocity = self.k_angular * self.normalize_angle(angle_to_goal - self.theta)

            linear_velocity = 0.0
            angular_velocity = 0.0

            if (self.trajectory_state == ''):

                rospy.loginfo("empty")

                linear_velocity = 0.0
                angular_velocity = 0.0
                self.trajectory_state = 'x'

            elif (self.trajectory_state == 'x'):

                rospy.loginfo("x")

                x_error = (self.x_goal - self.x)
                print("Error X:", x_error)

                if (x_error <= 0.05):
                    linear_velocity = 0.0
                    self.trajectory_state = 'z_1'
                elif (x_error > 0):
                    linear_velocity = 0.3
                elif (x_error < 0):
                    linear_velocity = -0.3

            elif (self.trajectory_state == 'z_1'):

                rospy.loginfo("z_1")

                orientation = self.x_goal - self.x

                if ((orientation) > 0):
                    angular_velocity = 0.5
                elif (orientation < 0):
                    angular_velocity = -0.5
                else:
                    self.trajectory_state = 'y'

                self.turn_counter += 1
                if self.turn_counter == 35:

                    angular_velocity = 0.0
                    #self.turn_counter = 0
                    self.trajectory_state = 'y'

            elif (self.trajectory_state == 'y'):

                rospy.loginfo("y")

                y_error = self.y_goal - self.y
                print("Error Y:", y_error)

                if (y_error <= 0.05):
                    linear_velocity = 0.0
                    self.trajectory_state = 'z_2'
                elif (y_error > 0):
                    linear_velocity = 0.3
                elif (y_error < 0):
                    linear_velocity = -0.3

            elif (self.trajectory_state == 'z_2'):

                rospy.loginfo("z_2")

                orientation = self.x_goal - self.x

                if ((orientation) > 0):
                    angular_velocity = -0.5
                elif (orientation < 0):
                    angular_velocity = 0.5
                else:
                    self.trajectory_state = ''

                self.turn_counter -= 1
                if self.turn_counter == 0:

                    angular_velocity = 0.0
                    #self.turn_counter = 0
                    self.trajectory_state = ''

        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity
        vel_msg.angular.z = angular_velocity
        self.velocity_publisher.publish(vel_msg)
        #print(dt)

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
        #x_desired = float(input("Desired x: "))
        #y_desired = float(input("Desired y: "))

        #controller = lyapunov_controller(x_desired, y_desired)
        controller = lyapunov_controller()
        controller.move_turtle()

    except rospy.ROSInterruptException:
        pass