#!/usr/bin/env python

import rospy
import time

# For Checking Python, OpenCV and Numpy
import platform
import cv2
import numpy as np

from speeds_assignment import RobotController

# The main function of the program
if __name__ == '__main__':


    # Wait for simulator and SLAM to initialize
    print "Waiting 10 seconds for initialization\n"
    time.sleep(10)

    # Print Versions of Python, OpenCV and Numpy
    print str('\n--------------------------------')
    print "Python version : {0}".format(platform.python_version())
    print "OpenCV version : {0}".format(cv2.__version__)
    print "Numpy version : {0}".format(np.__version__)
    print str('--------------------------------\n')

    # Initializes the ROS node
    rospy.init_node('turtlebot_controller')
    # Creates a RobotController object
    rc = RobotController()
    # ROS waits for events
    rospy.spin()
