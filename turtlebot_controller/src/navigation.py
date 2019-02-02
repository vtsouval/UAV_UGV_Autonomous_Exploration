#!/usr/bin/env python

import rospy
import math
import time
import numpy as np
import scipy.misc

from scipy.spatial import distance
from math import atan2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from robot_perception import RobotPerception
from target_selection import TargetSelection
from path_planning import PathPlanning
from utilities import RvizHandler, Print, TimerThread, ImageMatching

# For OGM Enhancement
import cv2
from cv_bridge import CvBridge, CvBridgeError
from skimage import exposure
from sensor_msgs.msg import Image
from utilities import OgmOperations
from numba import jit

# Costume Services and Messages Added
from drone_controller.srv import OgmLimits, OgmLimitsRequest


# Class for implementing the navigation module of the robot
class Navigation:

    # Constructor
    def __init__(self):

        # Initializations of Objects Needed
        self.robot_perception = RobotPerception()
        self.path_planning = PathPlanning()

        # Parameters from YAML File
        self.move_with_target = rospy.get_param("calculate_target")
        self.target_selector = rospy.get_param("target_selector")
        self.turtlebot_path_topic = rospy.get_param('turtlebot_path_topic')
        self.turtlebot_subgoals_topic = rospy.get_param('turtlebot_subgoals_topic')
        self.turtlebot_curr_target_topic = rospy.get_param('turtlebot_curr_target_topic')
        self.map_frame = rospy.get_param('map_frame')
        self.debug = rospy.get_param('debug')
        self.turtlebot_save_progress_images = rospy.get_param('turtlebot_save_progress_images')
        self.limit_exploration_flag = rospy.get_param('limit_exploration') # Flag to Enable/ Disable OGM Enhancement

        # Flag to check if the vehicle has a target or not
        self.target_exists = False
        self.inner_target_exists = False

        # Container for the current path
        self.path = []
        # Container for the subgoals in the path
        self.subtargets = []

        # Container for the next subtarget. Holds the index of the next subtarget
        self.next_subtarget = 0
        self.count_limit = 100 # 20 sec
        self.counter_to_next_sub = self.count_limit

        # Timer Thread (initialization)
        self.time_limit = 150 # in seconds
        self.timerThread = TimerThread(self.time_limit)

        # Check if subgoal is reached via a timer callback
        rospy.Timer(rospy.Duration(0.10), self.checkTarget)

        # Read the target function
        self.target_selection = TargetSelection(self.target_selector)

        # ROS Publisher for the path
        self.path_publisher = rospy.Publisher(self.turtlebot_path_topic, Path, queue_size=10)
        # ROS Publisher for the subtargets
        self.subtargets_publisher = rospy.Publisher(self.turtlebot_subgoals_topic, MarkerArray, queue_size=10)

        # ROS Publisher for the current target
        self.current_target_publisher = rospy.Publisher(self.turtlebot_curr_target_topic, Marker, queue_size=10)

        # For Testing Purpose
        self.timer_flag = True  # True to Enable Timer for resetting Navigation

        # Speed Parameter
        self.max_linear_velocity = rospy.get_param('max_linear_speed')
        self.max_angular_velocity = rospy.get_param('max_angular_speed')

        # Parameters of OGM Enhancement
        self.first_run_flag = True  # Flag to Control first Enhancement in OGM Map

        # Map Container Visualize Message
        self.map_size = np.array([[rospy.get_param('x_min_size'), rospy.get_param('y_min_size')],
                                       [rospy.get_param('x_min_size'), rospy.get_param('y_max_size')],
                                       [rospy.get_param('x_max_size'), rospy.get_param('y_min_size')],
                                       [rospy.get_param('x_max_size'), rospy.get_param('y_max_size')]], dtype=np.float64)

        if self.limit_exploration_flag:

            # Create CV Bridge Object
            self.bridge = CvBridge()
            # Parameters
            self.drone_image = []       # Hold the Drone OGM Image converted in CV!
            self.ogm_in_cv = []         # Hold the OGM in CV Compatible Version
            self.drone_yaw = 0.0        # Holds the current Drone Yaw
            self.drone_map_pose = []    # Holds the current Drone Position in Map
            # Service
            self.drone_explore_limit_srv_name = rospy.get_param('drone_explore_limit_srv')
            self.limits_exploration_service = rospy.ServiceProxy(self.drone_explore_limit_srv_name, OgmLimits)
            # Subscriber
            self.drone_pub_image_topic = rospy.get_param('drone_pub_image_topic')
            self.drone_image_sub = rospy.Subscriber(self.drone_pub_image_topic, Image, self.drone_image_cb)
            # Flags
            self.done_enhancement = False   # Flag for Controlling OGM Enhancement Loop
            self.drone_take_image = False   # Flag for Controlling When to Read Image From Drone Camera
            # FIXME: Testing
            self.match_with_limit_pose = True   # True/False to Match with Known Location/Template Matching
            self.match_with_drone_pose = False  # True if you match with Drone Real Pose

    def checkTarget(self, event):

        # Check if we have a target or if the robot just wanders
        if not self.inner_target_exists or not self.move_with_target or self.next_subtarget == len(self.subtargets):
            return

        # Check if timer has expired
        if self.timer_flag:
            if self.timerThread.expired:
                Print.art_print('\n~~~~ Time reset ~~~~',Print.RED)
                self.inner_target_exists = False
                self.target_exists = False
                return

        # Get the robot pose in pixels
        [rx, ry] = [self.robot_perception.robot_pose['x_px'] - self.robot_perception.origin['x'] / self.robot_perception.resolution,
                    self.robot_perception.robot_pose['y_px'] - self.robot_perception.origin['y'] / self.robot_perception.resolution]
        theta_robot = self.robot_perception.robot_pose['th']

        # Clear achieved targets
        self.subtargets = self.subtargets[self.next_subtarget:]
        self.next_subtarget = 0

        # If distance between the robot pose and the next subtarget is < 7 pixel consider that Target is Reached
        dist = math.hypot(rx - self.subtargets[self.next_subtarget][0], ry - self.subtargets[self.next_subtarget][1])
        if dist < 7:
            self.next_subtarget += 1
            self.counter_to_next_sub = self.count_limit
            # Check if the final subtarget has been approached
            if self.next_subtarget == len(self.subtargets):
                self.target_exists = False

        # Publish the current target
        if self.next_subtarget >= len(self.subtargets):
            return

        subtarget = [self.subtargets[self.next_subtarget][0] * self.robot_perception.resolution + self.robot_perception.origin['x'],
                     self.subtargets[self.next_subtarget][1] * self.robot_perception.resolution + self.robot_perception.origin['y']]

        RvizHandler.printMarker([subtarget],1, 0, "map", "art_next_subtarget", [0, 0, 0.8, 0.8], 0.2)

    # Function that selects the next target, produces the path and updates
    # the coverage field. This is called from the speeds assignment code, since
    # it contains timer callbacks
    def selectTarget(self):

        # IMPORTANT: The robot must be stopped if you call this function until
        # it is over

        # Cancel previous goal timer
        self.timerThread.stop()
        # Check if we have a map
        while not self.robot_perception.have_map:
            Print.art_print("Navigation: No map yet", Print.RED)
            return

        print "Clearing all markers !\n\n"
        RvizHandler.printMarker([[0, 0]],1, 3, "map", "null", [0,0,0,0], 0.1)
        # Visualize Map Container
        map_container_mark = []
        for s in self.map_size:
            map_container_mark.append([s[0]*self.robot_perception.resolution + self.robot_perception.origin['x'],
                                       s[1]*self.robot_perception.resolution + self.robot_perception.origin['y']])
        RvizHandler.printMarker(map_container_mark, 1, 0, "map", "art_map_container", [1.0, 0.0, 0.0, 1.0],0.3)

        print '----------------------------------------------------------'
        print 'Navigation: Producing new target'
        # We are good to continue the exploration
        # Make this true in order not to call it again from the speeds assignment
        self.target_exists = True

        ########################################################################
        # Get OGM Map and Coverage
        start_ = time.time()

        # Gets copies of the map and coverage
        start = time.time()
        local_ogm = self.robot_perception.getMap()
        if self.debug:
            print str('Robot Perception: Got the map in ') + str(time.time() - start) + str(' seconds.')

        start = time.time()
        local_ros_ogm = self.robot_perception.getRosMap()
        if self.debug:
            print str('Robot Perception: Got the ros map in ') + str(time.time() - start) + str(' seconds.')

        start = time.time()
        local_coverage = self.robot_perception.getCoverage()
        if self.debug:
            print str('Robot Perception: Got the coverage in ') + str(time.time() - start) + str(' seconds.')

        print str('Navigation: Got the map and Coverage in ') + str(time.time() - start_) + str(' seconds.')

        ########################################################################
        # Part 1 - OGM Enhancement
        no_points = False
        ogm_limits_before = []

        if self.turtlebot_save_progress_images:
            ####################### Save OGM and Coverage ##########################
            scipy.misc.imsave('/home/vvv/pictures_slam/ogm.png',
                              cv2.bitwise_not(exposure.rescale_intensity(np.array(local_ogm, dtype=np.uint8),
                                                                         in_range=(0, 100), out_range=(0, 255))))
            scipy.misc.imsave('/home/vvv/pictures_slam/coverage.png', local_coverage)

        start_ = time.time()

        print str(self.robot_perception.percentage_explored)

        if self.limit_exploration_flag and self.robot_perception.percentage_explored < 1.0  and not self.first_run_flag:

            #############################################################################
            # Subpart 1 - Find the Useful boundaries of OGM
            #############################################################################
            start = time.time()
            (points, ogm_limits_before) = self.ogm_limit_calculation(local_ogm, self.robot_perception.resolution,
                                                                     self.robot_perception.origin, 20, 20, 12, self.map_size)
            if points is None or len(points) == 0:
                print str('No Limits Points Calculated ')
                no_points = True
            else:
                print str('Number of Limit Points Calculated is ') + str(len(points)) + str(' in ')\
                      + str(time.time() - start) + str(' seconds.')
                no_points = False

            if self.debug:
                print str('\n') + str(points) + str('\n')
            #############################################################################

            #########################################################################
            # Subpart 2 - Send Drone to Limits and Match the to OGM Map
            #########################################################################
            startt = time.time()

            if not no_points:

                index = 0
                p = points[index, :]
                while len(points):

                    points = np.delete(points, [index], axis=0)     # Delete Point thats Going to be Explored

                    # Send Drone to OGM Limits
                    start = time.time()
                    while not self.sent_drone_to_limit(p[0], p[1]):
                        pass

                    if self.debug:
                        Print.art_print(str('Navigation: Get Image From Drone took ')
                                        + str(time.time() - start) + str(' seconds.'), Print.ORANGE)

                    # Image Matching
                    start = time.time()

                    if self.match_with_limit_pose:

                        if self.match_with_drone_pose:
                            [x_p, y_p] = [int((self.drone_map_pose[0] - self.robot_perception.origin['x'])
                                              / self.robot_perception.resolution),
                                          int((self.drone_map_pose[1] - self.robot_perception.origin['y'])
                                              / self.robot_perception.resolution)]

                            local_ogm = ImageMatching.ogm_match_with_known_image_pose(ogm=local_ogm, new_data=self.drone_image,
                                                                                      coordinates=[x_p, y_p, self.drone_yaw],
                                                                                      map_boundries=[self.map_size[0][0], self.map_size[3][0],
                                                                                                     self.map_size[0][1], self.map_size[3][1]],
                                                                                      debug=self.debug)
                        else:
                            local_ogm = ImageMatching.ogm_match_with_known_image_pose(ogm=local_ogm, new_data=self.drone_image,
                                                                                      coordinates=[int(p[2]), int(p[3]), self.drone_yaw],
                                                                                      map_boundries=[int(self.map_size[0][0]), int(self.map_size[3][0]),
                                                                                                     int(self.map_size[0][1]), int(self.map_size[3][1])],
                                                                                      debug=self.debug)

                    else:
                        local_ogm = ImageMatching.template_matching(ogm=local_ogm, drone_image=self.drone_image,
                                                                    lim_x=int(p[2]), lim_y=int(p[3]),
                                                                    drone_yaw=self.drone_yaw, window=200,
                                                                    s_threshold=0.8, debug=self.debug)

                    if self.debug:
                        Print.art_print(str('Navigation: OGM Matching Function  took ')
                                        + str(time.time() - start) + str(' seconds.\n'), Print.ORANGE)

                    # Calculate Point for Next Loop!
                    if len(points) > 0:
                        index = self.closest_limit_point(p[:2], points[:, :2])
                        p = points[index, :]
                '''
                print('\x1b[35;1m' + str('Navigation: Taking Image and Matching took ') +
                      str(time.time() - startt) + str(' seconds.') + '\x1b[0m')
                '''
            ########################################################################

            ########################################################################
            # Subpart 3 - -Copy Enhanced Data to ROS OGM
            ########################################################################
            if not no_points:

                start = time.time()
                local_ros_ogm.data = cp_data_to_ros_ogm(np.array(local_ros_ogm.data), local_ogm,
                                                        local_ros_ogm.info.width, local_ros_ogm.info.height)
                if self.debug:
                    print('\x1b[35;1m' + str('Navigation: Copying OGM Data took ') +
                          str(time.time() - start) + str(' seconds.') + '\x1b[0m')
            ########################################################################

        print('\x1b[38;1m' + str('Navigation: OGM Enhancement took ') +
              str(time.time()-start_) + str(' seconds.') + '\x1b[0m')

        if self.turtlebot_save_progress_images and self.limit_exploration_flag and not no_points:
            ########################## Save Enhance OGM ############################
            scipy.misc.imsave('/home/vvv/pictures_slam/ogm_enhanced.png',
                              cv2.bitwise_not(exposure.rescale_intensity(np.array(local_ogm, dtype=np.uint8),
                                                                         in_range=(0, 100), out_range=(0, 255))))

        ########################################################################
        # Part 2 - Set Map, Get Robot Position and Choose Target

        start = time.time()
        self.target_selection.path_planning.setMap(local_ros_ogm)
        print str('Navigation: Set Map in ') + str(time.time() - start) + str(' seconds.')

        # Once the target has been found, find the path to it
        # Get the global robot pose
        start = time.time()
        g_robot_pose = self.robot_perception.getGlobalCoordinates([self.robot_perception.robot_pose['x_px'],
                                                                   self.robot_perception.robot_pose['y_px']])
        print str('Navigation: Got Robot Pose in ') + str(time.time() - start) + str(' seconds.')

        # Call the target selection function to select the next best goal
        self.path = []
        force_random = False
        while len(self.path) == 0:

            # Get Target and Path to It!
            start = time.time()
            self.path = self.target_selection.selectTarget(local_ogm, local_coverage,
                                                        self.robot_perception.robot_pose, self.robot_perception.origin,
                                                        self.robot_perception.resolution, g_robot_pose, ogm_limits_before, force_random)
            #self.target_selection.path_planning.createPath(g_robot_pose, target, self.robot_perception.resolution)

            if len(self.path) == 0:
                Print.art_print('Navigation: Path planning failed. Fallback to random target selection', Print.RED)
                print str('\n')
                force_random = True
            else:
                print str('Navigation: Target Selected and Path to Target found with ') + str(len(self.path)) \
                      + str(' points in ') + str(time.time() - start) + str(' seconds.')

        ########################################################################
        # Part 3 - Create Subgoals and Print Path to RViz

        start = time.time()

        # Reverse the path to start from the robot
        self.path = self.path[::-1]
        # Break the path to subgoals every 2 pixels (1m = 20px)
        step = 1
        n_subgoals = int(len(self.path)/step)
        self.subtargets = []
        for i in range(0, n_subgoals):
            self.subtargets.append(self.path[i * step])
        self.subtargets.append(self.path[-1])
        self.next_subtarget = 0

        if self.debug:
            print str('Navigation: The path produced ') + str(len(self.subtargets)) + str(' subgoals in ') \
                  + str(time.time() - start) + str('seconds.')

        # Start timer thread
        self.timerThread.start()

        # Publish the path for visualization purposes
        ros_path = Path()
        ros_path.header.frame_id = self.map_frame
        for p in self.path:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = 0
            ps.pose.position.y = 0
            ps.pose.position.x = p[0] * self.robot_perception.resolution + self.robot_perception.origin['x']
            ps.pose.position.y = p[1] * self.robot_perception.resolution + self.robot_perception.origin['y']
            ros_path.poses.append(ps)
        self.path_publisher.publish(ros_path)

        # Publish the subtargets for visualization purposes
        subtargets_mark = []
        for s in self.subtargets:
            subtargets_mark.append([s[0] * self.robot_perception.resolution + self.robot_perception.origin['x'],
                                    s[1] * self.robot_perception.resolution + self.robot_perception.origin['y']])
        RvizHandler.printMarker(subtargets_mark, 2, 0, 'map', 'art_subtargets', [0, 0.8, 0.0, 0.8], 0.2)

        # Update NUmber of Targets
        self.robot_perception.num_of_explored_targets += 1

        self.inner_target_exists = True

        # Reset First Run Flag
        if self.first_run_flag:
            self.first_run_flag = False

    # Function that calculates Velocity to next Target
    def velocitiesToNextSubtarget(self):

        # Initialize Speeds
        [linear, angular] = [0, 0]
        # Get Robot Position in Map
        [rx, ry] = [self.robot_perception.robot_pose['x_px'] - self.robot_perception.origin['x'] / self.robot_perception.resolution, \
                    self.robot_perception.robot_pose['y_px'] - self.robot_perception.origin['y'] / self.robot_perception.resolution]
        theta = self.robot_perception.robot_pose['th']
        # If Target Exists Calculate Speeds
        if self.subtargets and self.next_subtarget <= len(self.subtargets) - 1:
            # Calculate dx, dy, dth
            st_x = self.subtargets[self.next_subtarget][0]
            st_y = self.subtargets[self.next_subtarget][1]

            th_rg = np.arctan2(st_y - ry, st_x - rx)
            dth = (th_rg - theta)

            if dth > np.pi:
                omega = (dth - 2 * np.pi) / np.pi
            elif dth < -np.pi:
                omega = (dth + 2 * np.pi) / np.pi
            else:
                omega = (th_rg - theta) / np.pi

            # Nonlinear Relations Derived from Experimentation
            linear = self.max_linear_velocity * ((1 - np.abs(omega)) ** 5)
            angular = self.max_angular_velocity * np.sign(omega) * (abs(omega) ** (1 / 5))

            '''
            new_theta = atan2(st_y - ry, st_x - rx)
            if new_theta <= -3:
                new_theta = -new_theta
            if theta <= -3:
                theta = -theta
            delta_theta = new_theta - theta
            if abs(delta_theta) > 3.15:
                delta_theta = -0.2 * np.sign(delta_theta)
            # Assign Speeds Accordingly
            if abs(delta_theta) < 0.1:
                linear = self.max_linear_velocity
                angular = 0
            else:
                linear = 0
                angular = self.max_angular_velocity*np.sign(delta_theta)
            '''
        # Return Speeds
        return [linear, angular]

    # Send SIGNAL to Drone to Explore Limit Function
    def sent_drone_to_limit(self, x, y):

        # Set Flag to Make Drone Image to be Received
        self.drone_take_image = True

        # Create Request with Points in Map of OGM Limits
        request = OgmLimitsRequest()
        request.x = x
        request.y = y

        # Call Service
        rospy.wait_for_service(self.drone_explore_limit_srv_name)
        try:
            response = self.limits_exploration_service(request)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        # Wait for Drone Image to be Received to Return
        while self.drone_take_image:
            pass

        # Return Once Image has been Received
        return response

    # Callback Function that Save Drone Image in CV
    def drone_image_cb(self, image):

        if self.drone_take_image:
            # Save Rotation and Position in Current Time
            self.drone_yaw = self.target_selection.droneConnector.get_drone_yaw()
            if self.match_with_drone_pose:
                self.drone_map_pose = np.array(self.target_selection.droneConnector.get_drone_pose(), dtype=np.float64)
            # Save Received Image and Convert it to Black and White
            try:
                image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            except CvBridgeError as e:
                print(e)
            img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(img_hsv, np.array([0, 50, 50]), np.array([10, 255, 255])) \
                   + cv2.inRange(img_hsv, np.array([170, 50, 50]), np.array([180, 255, 255]))
            image[np.where(mask == 0)] = [255, 255, 255]
            image[np.where(mask == 255)] = [0, 0, 0]
            self.drone_image = image
            # Reset Flag to Continue
            self.drone_take_image = False

    # Closest Next Point to the Drone Calculation Function
    @staticmethod
    def closest_limit_point(point, points):
        return np.argmin(distance.cdist(point.reshape(1, -1), points, 'euclidean'))

    # Calculate OGM Limit Points Function (in Map Position and Pixel Size)
    @staticmethod
    def ogm_limit_calculation(ogm, resolution, origin, find_step=20, search_step=20, max_points=10, map_size=[]):

        (ogm_lp, previous_ogm_limits) = OgmOperations.findLimitPoints(ogm, origin, resolution, find_step, search_step, max_points, map_size)
        if ogm_lp is None:
            return None
        else:
            ogm_lm = np.array(ogm_lp, dtype=np.float64)
            for n in ogm_lm:
                n[0] = n[0]*resolution + origin['x']
                n[1] = n[1]*resolution + origin['y']
        return np.array(np.concatenate((ogm_lm[:,:2], ogm_lp), axis=1), dtype=np.float64), previous_ogm_limits


# Faster Loops in Native C using Numba
@jit(nopython=True, parallel=True)
def cp_data_to_ros_ogm(ros_ogm, ogm, width, height):

    for i in range(0, width):
        for j in range(0, height):
            ros_ogm[i + width * j] = ogm[i][j]

    return ros_ogm


