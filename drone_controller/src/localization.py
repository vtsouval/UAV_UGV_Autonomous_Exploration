#!/usr/bin/env python

import rospy
import numpy as np

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist
# Custom Messages and Services Added!
from drone_controller.srv import JointTrees, JointTreesRequest


class Localization:

    # Constructor
    def __init__(self):

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Flag Needed
        self.localized = False # Flag that hold if Drone has been Localized or Not!
        self.tag_is_found = False # Set to True if Tag is Found!

        # FIXME: Parameter from YAML File
        self.drone_speed_topic = rospy.get_param('drone_speed_topic')
        self.camera_frame = rospy.get_param('drone_camera_frame')
        self.tag_frame = rospy.get_param('ar_pose_frame')
        self.join_tree_srv_name = rospy.get_param('join_tree_srv')
        self.localization_height = rospy.get_param('localization_height')
        self.exploration_height = rospy.get_param('exploration_height')

        # Twist Creation
        self.l_speed = 0.0
        self.r_speed = 0.0
        self.vel_msg_l = Twist()
        self.vel_msg_r = Twist()
        self.vel_msg_e = Twist()
        self.initialize_speed()

        # Service Subscriber
        self.join_tree_srv = rospy.ServiceProxy(self.join_tree_srv_name, JointTrees)
        # Drone Speed Publish Topic
        self.pub_cmd = rospy.Publisher(self.drone_speed_topic, Twist, queue_size=10)
        # Check if Tag is Found via a Timer
        self.check_tag_timer = rospy.Timer(rospy.Duration(0.10), self.check_tag_found)

    # Function that Moves Drone in Map to Find Drone
    def search_for_tag_flag(self):

        # Initial Distance
        distance = 1.0
        found = False
        # While tag is not Found Send Speed to Drone
        while not self.tag_is_found:
            # Initialize found Flag and Set Distance to Cover
            print str('Searching in a Square Area of ') + str(distance)
            # Publish Right Speeds to Travel in Square Area!
            for x in range(0, 3):
                # Return true if Tag if found!
                if self.move_line(distance):
                    found = True
                    break
                # Return true if Tag if found!
                if self.rotate_angle(90.0):
                    found = True
                    break

            # Not Found in this Area, Make Wider Search
            distance += 2.0

            # Set True if Tag if found, so break from Loop!
            if found:
                break
        # Now that tag is found Join The Trees!
        if self.tag_is_found:
            self.join_drone_to_tree()
        # Stop The Timer
        self.check_tag_timer.shutdown()
        # Return True to Let Us Know

        return True

    # Function that Initialize Twists
    # Linear is [Meters/Second] and Rotational is [Degrees/Second]
    def initialize_speed(self, clockwise=True, straight=True, l_speed=0.75, r_speed=20.0):
        # Set Speeds
        self.l_speed = l_speed
        self.r_speed = r_speed
        # Create Linear Twist
        if straight:
            self.vel_msg_l.linear.x = abs(self.l_speed)
        else:
            self.vel_msg_l.linear.x = -abs(self.l_speed)
        # Create Rotate Twist
        if clockwise:
            self.vel_msg_r.angular.z = -abs(self.r_speed*(2*np.pi/360))
        else:
            self.vel_msg_r.angular.z = abs(self.r_speed*(2*np.pi/360))

    # Function that Moves Drone to Straight Distance
    def move_line(self, distance):

        # Get Initial Time
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        # Loop to move the Drone to Specified Distance
        while current_distance < distance:
            # Publish the velocity
            self.pub_cmd.publish(self.vel_msg_l)
            # Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            # Calculates Distance
            current_distance = self.l_speed*(t1-t0)
            # Check if Tag is Found
            if self.tag_is_found:
                self.pub_cmd.publish(self.vel_msg_e)
                return True

        # Force the robot to stop
        self.pub_cmd.publish(self.vel_msg_e)
        return False

    # Function that Rotate Drone to an Angle
    def rotate_angle(self, relative_angle):

        # Get Initial Time
        t0 = rospy.Time.now().to_sec()
        relative_angle *= 2*np.pi/360
        current_angle = 0
        # Loop to move the Drone to Specified Angle
        while current_angle < relative_angle:
            # Publish the velocity
            self.pub_cmd.publish(self.vel_msg_r)
            # Takes actual time to velocity calculus
            t1 = rospy.Time.now().to_sec()
            # Calculates Angle
            current_angle = (self.r_speed*(2*np.pi/360))*(t1 - t0)
            # Check if Tag is Found
            if self.tag_is_found:
                self.pub_cmd.publish(self.vel_msg_e)
                return True

        # Force the robot to stop
        self.pub_cmd.publish(self.vel_msg_e)
        # False if Flag is not Found
        return False

    # Function that Send SIGNAL to Join the Drone to Main Tree
    def join_drone_to_tree(self):

        # Create Request with Points in Map of OGM Limits
        request = JointTreesRequest()
        request.status = True
        # Call Service
        rospy.wait_for_service(self.join_tree_srv_name)
        try:
            response = self.join_tree_srv(request)
            if response:
                self.localized = True
            else:
                self.localized = False
        except rospy.ServiceException, e:
            self.localized = False
            print "Service call failed: %s" % e

    # Function that Check if we can Calculate the TF between Drones Bottom Camera and Ar Tag
    def check_tag_found(self, event):
        if self.tf_buffer.can_transform(self.tag_frame, self.camera_frame, rospy.Time(0), rospy.Duration(1.0)) and \
                not self.tag_is_found:
            self.tag_is_found = True

