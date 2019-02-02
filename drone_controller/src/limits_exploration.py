#!/usr/bin/env python

import rospy
import numpy as np
from time import sleep
from sensor_msgs.msg import Image
from utilities import Plane_PID
# Custom Messages and Services Added!
from drone_controller.srv import OgmLimits


# Class for Limits Exploration
class Limits_Exploration:

    # Constructor
    def __init__(self):

        # Create Plane PID Object
        self.pid = Plane_PID()
        #self.pid.initialize_gains(kp=0.3,ki=0.0,kd=0.01,error=0.15)
        self.pid.initialize_gains(kp=0.4,ki=0.0,kd=0.04,error=0.15)


        # Parameter from YAML File
        self.debug = rospy.get_param('debug')
        self.drone_camera_topic = rospy.get_param('drone_camera_topic')
        self.drone_pub_image_topic = rospy.get_param('drone_pub_image_topic')
        self.drone_explore_limit_srv = rospy.get_param('drone_explore_limit_srv')

        # Flags
        self.drone_limit_exploration_flag = False
        self.take_image = False             # Flag to Control When to Save Image from Bottom Camera

        # Parameters
        self.limit_position = [0,0]         # Parameter that Holds The Position of Limit
        self.drone_limit_image = Image()    # Parameter that holds Image from Bottom Camera for OGM Enhancement

        # Topic that will publish the Image for OGM Enhancement
        self.drone_image_publisher = rospy.Publisher(self.drone_pub_image_topic, Image, queue_size=1)
        # Subscribe to ArDrone Bottom Camera
        self.drone_camera_subscriber = rospy.Subscriber(self.drone_camera_topic, Image, self.drone_camera_cb)
        # Service Responsible for Limit Exploration
        self.limits_exploration_service = rospy.Service(self.drone_explore_limit_srv,
                                                        OgmLimits, self.handle_limit_exploration_srv_function)

    # Limits Exploration Service Handle Function
    def handle_limit_exploration_srv_function(self, limits):

        print ('\x1b[38;1m' + '\nLimits Exploration Service Called' + '\x1b[0m')

        # Set Limit Exploration Flag
        self.drone_limit_exploration_flag = True
        # Create Array that holds the Position to be Reached
        self.limit_position = np.array([[limits.x, limits.y]], dtype=np.float64).reshape(1,-1)
        # For Debug Purposes
        if self.debug:
            print str('Going to OGM Limits: [') + str(limits.x) + str(', ') + str(limits.y) + str('].')
        # Run the Function to Explore OGM Limit
        self.drone_image_publisher.publish(self.drone_limit_exploration())
        # For Debug Purposes
        if self.debug:
            print str('Reached the Limit!')
        # Reset Limit Exploration Flag
        self.drone_limit_exploration_flag = False
        # Return Success Status
        return True

    # Exploration Limit Function
    def drone_limit_exploration(self):

        # Travel to Limit
        while not self.pid.sent_drone_to_position(self.limit_position[0, 0], self.limit_position[0, 1], 0):
            pass
        # Sleep to make sure Drone is Stabilized
        sleep(0.1)
        # Take Image from OGM Limits!
        self.take_image = True
        while self.take_image:
            pass
        return self.drone_limit_image

    # Callback Function for Drone Bottom Camera Topic that Saves Image at the Right Time
    def drone_camera_cb(self, image):

        if self.take_image:
            self.drone_limit_image = image
            self.take_image = False

