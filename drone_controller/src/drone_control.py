#!/usr/bin/env python

import rospy
from rosnode import kill_nodes
from time import sleep
from math import fabs
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from utilities import Marker_Follower
from localization import Localization
from color_evaluation import Color_Evaluation
from limits_exploration import Limits_Exploration

# Custom Messages and Services Added!
from drone_controller.srv import ControlDrone


# TODO: Class for Control_Drone
class Control_Drone:

    # Constructor
    def __init__(self):

        # Create Marker Follower Object
        self.marker_follower = Marker_Follower()
        self.marker_follower.initialize_gains() # Initialize PID with Default Gains
        # Create a Localization object
        self.localization = Localization()
        # Create a Color Evaluation object
        self.color_evaluation = Color_Evaluation()
        # Create a Target Create object
        self.limits_exploration = Limits_Exploration()

        # FIXME: Parameters from YAML File
        self.height_down = rospy.get_param('low_height')        # Used in fly_to() Function
        self.height_up = rospy.get_param('exploration_height')  # Used in fly_to() Function
        self.drone_speed_topic = rospy.get_param('drone_speed_topic')
        self.drone_takeoff_topic = rospy.get_param('drone_takeoff_topic')
        self.drone_land_topic = rospy.get_param('drone_land_topic')
        self.debug = rospy.get_param('debug')

        # Service Responsible for Color Evaluation of Potential Targets
        self.drone_control_srv_name = rospy.get_param('drone_control_srv')
        self.drone_control_service = rospy.Service(self.drone_control_srv_name,
                                                   ControlDrone, self.drone_control_srv_function)
        # Drone Speed/Takeoff/Land Publish Topics
        self.pub_cmd = rospy.Publisher(self.drone_speed_topic, Twist, queue_size=10)
        self.takeoff = rospy.Publisher(self.drone_takeoff_topic, Empty, queue_size=100)
        self.land = rospy.Publisher(self.drone_land_topic, Empty, queue_size=100)

        # Flags Needed for Drone Control
        self.marker_follow_flag = False      # Flag to start/stop PID to Follow Marker
        self.fly_flag = False                # Flag to Control Drone to fly Up/Down
        self.recovery_behavior = False  # Used in recovery() Function if Marker is Lost
        self.fly_direction = ''         # Holds the Direction that Drone is Flying (Up/Down)
        self.state = False              # Falg to check if Drone has Taken Off Ground and False if Drone has Landed
        self.take_image = False         # True if Needed to Take Image from Drone Bottom Camera

        # Parameters
        self.empty = Twist()
        self.taken_image = Image()      # Hold the Image Taken from Drone Bottom Camera

    '''
    IF command = 1 -> Start_PID
    IF command = 2 -> Stop_PID
    IF command = 3 -> Go_UP
    IF command = 4 -> Go_DOWN
    IF command = 5 -> Takeoff
    IF command = 6 -> Land
    IF command = 7 -> Check if Drone if Following the Tag
    ELSE : Invalid Command Received
    '''
    # Drone Control Service Handle Function
    def drone_control_srv_function(self, received):

        if self.debug:
            print ('\x1b[38;1m' + str('Drone Control Service has been called with Command Number ')
                   + str(received.command) + str('.\n')
                   + str('PID Current State is ') + str(self.marker_follow_flag) + str('.\n')
                   + str(' Target is ') + str(self.can_Transform() and self.check_tag_exists()) + str('.\n')
                   + str('Target Reach is: ') + str(self.target_is_reached()) + str('.\n') + '\x1b[0m')
        else:
            print ('\x1b[38;1m' + str('Drone Control Service has been called with Command Number ')
                   + str(received.command) + '\x1b[0m')

        # Set Flag According to Command Received
        if received.command == 1:
            if not self.marker_follow_flag and not self.fly_flag:
                self.marker_follow_flag = True
                return 'Done'
            else:
                if self.marker_follow_flag:
                    return 'PID Already ON'
                else:
                    if self.fly_direction == 'Go_UP':
                        return 'Drone Going Up'
                    elif self.fly_direction == 'Go_DOWN':
                        return 'Drone Going Down'
                    else:
                        return self.fly_direction

        elif received.command == 2:
            if self.marker_follow_flag:
                self.marker_follow_flag = False
                return 'Done'
            else:
                return 'PID Already OFF'

        elif received.command == 3:
            if self.target_is_reached() and not self.fly_flag:
                self.marker_follow_flag = False
                self.fly_direction = 'Go_UP'
                self.fly_flag = True
                return 'Done'
            else:
                if self.fly_flag:
                    return 'Drone is Flying DOWN'
                else:
                    return 'Not in position'

        elif received.command == 4:
            if not self.marker_follow_flag and not self.fly_flag:
                self.fly_direction = 'Go_DOWN'
                self.fly_flag = True
                return 'Done'
            else:
                if self.marker_follow_flag:
                    return 'PID Running'
                else:
                    return 'Drone is Flying UP'

        elif received.command == 5:
            if self.drone_takeoff():
                return 'Done'
            else:
                return 'Drone Already Flying'

        elif received.command == 6:
            if self.drone_land():
                return 'Done'
            else:
                return 'Drone Already Landed'

        elif received.command == 7:
            if self.marker_follow_flag and self.check_tag_exists() and self.can_Transform():
                return 'Done'
            else:
                if not self.marker_follow_flag:
                    return 'PID is OFF'
                elif not self.check_tag_exists() and not self.can_Transform():
                    return 'No Current Tag'
                else:
                    return 'Retry'

        else:
            return 'Invalid Command'

    # Drone Takeoff Function
    def drone_takeoff(self):

        if not self.state:
            # Publish to Takeoff Topic
            time = rospy.get_time()
            while rospy.get_time() < time + 1.0:
                self.takeoff.publish(Empty())
            # Wait 2 seconds to Go Up
            sleep(2)
            # Check if Drone is Up
            if fabs(self.drone_height()) > 0.2:
                # Set State Flag
                self.state = True
                print ('\x1b[33;1m' + 'Drone Takeoff!' + '\x1b[0m')
                return True
            else:
                return False
        else:
            return False

    # Drone Land Function
    def drone_land(self):

        print ('\x1b[33;1m' + 'Drone Landing!' + '\x1b[0m')

        if self.state:
            # Publish to land service
            time = rospy.get_time()
            while rospy.get_time() < time + 1.0:
                self.land.publish(Empty())
            # Reset State Flag
            self.state = False
            # Wait 2 seconds to Go Down
            sleep(2)
            return True
        else:
            return False

    # Drone Function Returning the Current Height that Drone is Flying
    def drone_height(self):

        if self.marker_follower.tf.can_transform(self.marker_follower.drone_base_footprint,
                                                 self.marker_follower.drone_base_link,
                                                 rospy.Time(0), rospy.Duration(1.0)):

            h_tf = self.marker_follower.tf.lookup_transform(self.marker_follower.drone_base_footprint,
                                                            self.marker_follower.drone_base_link,
                                                            rospy.Time(0), rospy.Duration(1.0))
            return h_tf.transform.translation.z
        else:
            return 0

    # Drone Speed Publish Function
    def drone_speed_pub(self, speed):

        # Sent Speed to Drone for 0.1 Seconds
        time = rospy.get_time()
        while rospy.get_time() < time + 0.1:
            self.pub_cmd.publish(speed)
        self.pub_cmd.publish(self.empty)

    # Fly to X Meters Function
    def fly_to(self, command):

        # Determine Direction of Flight
        if command == 'Go_DOWN':
            sign = -1
            direction = 'DOWN'
            X = self.height_down
        else:
            sign = 1
            direction = 'UP'
            X = self.height_up

        # Initial Height and Flag for PID
        z = sign*self.drone_height()
        z_last = z
        first_run = True
        # Run Loop Till Height is Reached
        while fabs(fabs(z) - X) > 0.03:
            # Calculate Speed to get to Desired Height and Sent it to Drone Speed Topic
            if first_run:
                first_run = False
                self.marker_follower.initialize_pid()
            self.drone_speed_pub(self.marker_follower.pose_pid(0, 0, 0, 0, z, z_last, 0, 0))
            # Calculate New Height and Store Last One for Next Loop Run
            z_last = z
            z = sign * self.drone_height()
        self.pub_cmd.publish(self.empty)
        # For Debug Purposes
        if self.debug:
            print str("Gone ") + str(direction) + str(" to ") + str(fabs(z)) + str(" meters.")

        return True

    # Marker Tracking Function
    def marker_tracker(self):

        first_run = True
        x_last = y_last = z_last = yaw_last = 0

        # Stop if Command Sent or Another Service is Called
        while self.marker_follow_flag:
            # Check if any Service has been Called
            if self.color_evaluation.color_evaluate_targets_flag or\
                    self.limits_exploration.drone_limit_exploration_flag:
                self.marker_follow_flag = False
                break
            # Target ON, just Follow it
            if self.marker_follower.check_target_reached():

                # Get Pose Error
                [x, y,_, yaw,z] = self.marker_follower.calculate_error_from_tag()
                # Height of Flight is Constant in height_down Parameter
                z = -z + self.height_down
                # Run PID Initializer if First Time Flag is True
                if first_run:
                    self.marker_follower.initialize_pid()
                '''
                # X-Y are Reversed!
                # Pose PID Inputs Order: x,y,z,yaw
                # But now is : y,x,z,yaw
                # Drones Bottom Camera is a little forward at + 0.025
                # so get 0.1 back!
                '''
                # Publish Speed
                if not first_run:
                    self.drone_speed_pub(
                        self.marker_follower.pose_pid(y + 0.1, y_last + 0.1,
                                                      x, x_last,
                                                      z, z_last,
                                                      yaw, yaw_last))
                else:
                    first_run = False
                # Store Last Pose for Next Loop Run
                x_last = x
                y_last = y
                z_last = z
                yaw_last = yaw
            # Target OFF, start Recovery State
            else:
                self.drone_speed_pub(self.marker_follower.drone_marker_retrieval())

    # Drone Localization Function
    def drone_localize_in_map(self):

        # Get to Right Height
        self.height_up = self.localization.localization_height
        while not self.fly_to('Go_UP'):
            pass
        # Search for Tag
        self.localization.search_for_tag_flag()
        # Wait Till the Two Trees are Joint in One
        while not self.localization.localized:
            pass
        # Kill the Ar Pose Thread to save CPU Usage!
        kill_nodes(['/ar_pose'])
        # Get to Exploration Height
        self.height_up = self.localization.exploration_height
        while not self.fly_to('Go_UP'):
            pass

        print ('\x1b[33;1m' + 'Drone Localized in Map' + '\x1b[0m')

        return

    # COntrol Drone Decision with a Timer
    def drone_control(self, event):

        # Drone Takeoff if it hasn't already
        if not self.state:
            while not self.drone_takeoff():
                pass
        # Make sure Drone is Localized Before Doing Anything Else
        if not self.localization.localized:
            self.drone_localize_in_map()

        # Run Appropriate Command using the Flags

        if self.marker_follow_flag:
            self.marker_tracker()

        elif not self.limits_exploration.drone_limit_exploration_flag:

            if self.fly_flag:
                while not self.fly_to(self.fly_direction):
                    pass
                self.fly_direction = ''
                self.fly_flag = False

            elif self.color_evaluation.color_evaluate_targets_flag:
                self.color_evaluation.color_evaluation_result_pub(self.color_evaluation.evaluate_targets())