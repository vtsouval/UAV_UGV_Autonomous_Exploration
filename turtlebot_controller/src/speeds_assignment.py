#!/usr/bin/env python

import rospy
import math
import time
import numpy as np
from math import exp


from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds 
class RobotController:

    # Constructor
    def __init__(self):

        self.print_velocities = rospy.get_param('print_velocities')     # For Printing Velocity Calculated
        self.debug = rospy.get_param('debug')   # For Debugging
        self.move_with_target = rospy.get_param("calculate_target")     # Robot moves with target or just wanders
        self.initial_turn = rospy.get_param('initial_turn')     # Perform a 360 turn when starting

        # Where and when should you use this?
        self.stop_robot = False
        self.sonar_on = False

        # Create the needed Objects
        self.laser_aggregation = LaserDataAggregator()
        self.navigation = Navigation()
        if self.sonar_on:
            self.sonar_aggregation = SonarDataAggregator()  # Enable this if Robot has Sonar!

        # Speed Parameters for Turtlebot
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.low_turn_limit = 4.2
        self.max_linear_velocity = rospy.get_param('max_linear_speed')
        self.max_angular_velocity = rospy.get_param('max_angular_speed')

        # The Timer Produces Events for Sending Speeds Every 110 ms
        rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)

        # The Turtlebot Speed Publisher Topic
        self.velocity_publisher = rospy.Publisher(rospy.get_param('turtlebot_speeds_topic'),
                                                  Twist, queue_size = 10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):
        
        # Produce speeds
        self.produceSpeeds()
        # Create the commands message
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.angular_velocity
        # Send the command
        self.velocity_publisher.publish(twist)
        # Print the speeds for debuggind purposes
        if self.print_velocities:
            print "[L,R] = [" + str(twist.linear.x) + " , " + str(twist.angular.z) + "]"

    # Produces speeds from the laser
    def produceSpeedsLaser(self):

        scan = np.array(self.laser_aggregation.laser_scan)
        angle_min = self.laser_aggregation.angle_min
        angle_max = self.laser_aggregation.angle_max
        angles = np.linspace(angle_min, angle_max, len(scan))

        linear = -sum(np.cos(angles) / (scan ** 2)) / len(scan)
        angular = -sum(np.sin(angles) / (scan ** 2)) / len(scan)

        return [linear, angular]

    # Produces speeds from the sonars
    def produceSpeedsSonars(self):

        # Mask for sonar usage
        mask = np.array([0,0,0,1,1])  # only rear sonars config
        n = sum(mask)
        # Read Sonar Angles
        angles = np.array(self.sonar_aggregation.sonar_angles)
        # Read Scans
        ranges = [self.sonar_aggregation.sonar_front_range, self.sonar_aggregation.sonar_left_range,
                  self.sonar_aggregation.sonar_right_range, self.sonar_aggregation.sonar_rear_left_range,
                  self.sonar_aggregation.sonar_rear_right_range]
        # Normalization
        ranges = list(map(self.sonarNormalization,ranges))
        ranges = np.array(ranges)
        # Speed Calculation
        linear  = -sum(np.cos(angles)*ranges*mask)/n  # linear speed
        angular = -sum(np.sin(angles)*ranges*mask)/n  # angular speed
        # Normalization to robots allowed speeds
        linear = np.clip(linear, -self.max_linear_velocity, self.max_linear_velocity)
        angular = np.clip(angular, -self.max_angular_velocity, self.max_angular_velocity)

        return [linear, angular]

    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):

        if self.initial_turn:
            # Read and convert theta to (0, 2*pi)
            theta = self.navigation.robot_perception.robot_pose['th']
            if theta < 0:
                theta += 2*np.pi
            # Turn completed when theta is in (340,360) else, turn around!
            if self.low_turn_limit < theta < 2*np.pi-0.1:
                self.initial_turn = False
                if self.debug:
                    print str('Finish Initial Rotation!')
            else:
                self.angular_velocity = 0.3
                return

        # Produce target if not existent
        if self.move_with_target and not self.navigation.target_exists:

            # Create the commands message
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0

            # Send the command
            self.velocity_publisher.publish(twist)
            self.navigation.selectTarget()

        # Get the submodule's speeds
        [l_laser, a_laser] = self.produceSpeedsLaser()

        if self.sonar_on:
            # Get Sonar Speeds:
            [l_sonar, a_sonar] = self.produceSpeedsSonars()

        # Initialize Speeds
        self.linear_velocity = 0
        self.angular_velocity = 0

        # If we have a Target, get to it!
        if self.move_with_target:
            [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()

            if self.sonar_on:
                self.linear_velocity = l_goal + 0.1 * l_laser + l_sonar
                self.angular_velocity = a_goal + 0.25 * a_laser + a_sonar
            else:
                self.linear_velocity = l_goal + 0.1 * l_laser
                self.angular_velocity = a_goal + 0.25 * a_laser
        # Else use Laser to Wonder
        else:
            self.linear_velocity =l_laser
            self.angular_velocity = a_laser

        # Make sure velocities are in the desired range
        self.linear_velocity = np.clip(self.linear_velocity,
                                       -self.max_linear_velocity, self.max_linear_velocity)
        self.angular_velocity = np.clip(self.angular_velocity,
                                        -self.max_angular_velocity, self.max_angular_velocity)

    # Assistive functions
    def stopRobot(self):
        self.stop_robot = True

    def resumeRobot(self):
        self.stop_robot = False

    # Sonar Data Normalization
    def sonarNormalization(self,r):

        if r <= 0.15:
            return 1
        elif 0.15 < r <= 0.3:
            return -5.33333*r + 1.8
        elif 0.3 < r <= 0.5:
            return -1.5*(r - 0.5)
        else:
            return 0
