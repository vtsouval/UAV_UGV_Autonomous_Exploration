#!/usr/bin/env python

import rospy
import numpy as np
from math import hypot
from time import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from utilities import Plane_PID, Point_to_Pixel, Corner_Detection
from cv_bridge import CvBridge, CvBridgeError
from cv2 import inRange, cvtColor, COLOR_BGR2HSV
from numba import jit
# Custom Messages and Services Added!
from drone_controller.msg import ColorEvaluationArray
from drone_controller.srv import EvaluateTargets


# Class for Color_Weight_Calculation
class Color_Evaluation:

    # Constructor
    def __init__(self):

        # Parameter from YAML File
        self.debug = rospy.get_param('debug')
        self.drone_camera_topic = rospy.get_param('drone_camera_topic')
        self.drone_camera_frame = rospy.get_param('drone_camera_frame')
        self.drone_pub_color_rating = rospy.get_param('drone_pub_color_rating')

        # Flags
        self.color_evaluate_targets_flag = False
        self.take_image_and_tf_flag = False

        # Parameters
        self.drone_image = Image()          # Parameter that holds Image from Bottom Camera for Color Evaluation
        self.corner_image = Image()         # Parameter that holds Image from Bottom Camera for Corner Evaluation
        self.transform = TransformStamped() # Parameter that holds the TF from Map to Drone at Specific Moment
        self.targets = []                   # Parameter that holds the Position of Target for Evaluation
        self.evaluation_result = ColorEvaluationArray()     # Return Arrays of Color Evaluation Service

        # Create Plane PID Object
        self.pid = Plane_PID()
        #self.pid.initialize_gains(kp=0.3,ki=0.0,kd=0.01,error=0.15)
        self.pid.initialize_gains(kp=0.45, ki=0.0, kd=0.06, error=0.25)

        # Create Point to Pixel Object
        self.pixel_transform = Point_to_Pixel()
        # Create Corner Detection Object
        self.corner_detection = Corner_Detection()
        # CreateCvBridge Object
        self.bridge = CvBridge()

        # Topic that Color Evaluation Result will be Published
        self.color_evaluation_result_pub = rospy.Publisher(self.drone_pub_color_rating,
                                                             ColorEvaluationArray, queue_size=10)

        # Service Responsible for Color Evaluation of Potential Targets
        self.evaluate_potential_targets_srv = rospy.get_param('rate_potential_targets_srv')
        self.color_evaluation_service = rospy.Service(self.evaluate_potential_targets_srv,
                                                      EvaluateTargets, self.handle_evaluation_srv_function)

        # Subscribe to ArDrone Bottom Camera
        self.drone_camera_subscriber = rospy.Subscriber(self.drone_camera_topic,
                                                        Image, self.drone_camera_cb)

    # Potential Targets Evaluation Service Handle Function
    def handle_evaluation_srv_function(self, potential_targets):

        print ('\x1b[38;1m' + str('\n')
               + str('Potential Targets Evaluation Service Called') + str('\n') + '\x1b[0m')

        # Re-Construct Received Array in : [ ID | Pos_X | Pos_Y ]
        self.targets = np.concatenate((np.asarray(range(0, len(potential_targets.pos_x))).reshape(-1, 1),
                                       np.asarray([potential_targets.pos_x]).reshape(-1, 1),
                                       np.asarray([potential_targets.pos_y]).reshape(-1, 1)), axis=1)
        # Create Return Arrays in form of : [ Weight Value ], [ Corners Detected ]
        self.evaluation_result.color_weight = np.zeros((len(potential_targets.pos_x), 1), dtype=np.float64)
        self.evaluation_result.corner_number = np.zeros((len(potential_targets.pos_x), 1), dtype=np.int32)
        # Set Flag True and Return It!
        if not self.color_evaluate_targets_flag:
            self.color_evaluate_targets_flag = True
        return self.color_evaluate_targets_flag

    # Evaluate Potential Targets Function
    def evaluate_targets(self):

        start = time()
        #####################################################################################
        while len(self.targets):
            # Part 1 - Get Distance of Nodes From Drone and Store and Find Minimum One
            temp_targets = np.concatenate((self.targets,self.distance_from_targets(self.targets)), axis=1)
            (_, _, _, _, min_area) = temp_targets[np.argmin(temp_targets[:, 3]), :]
            # Part 2 - Create Sub-Area Array
            sub_area = np.array(temp_targets[temp_targets[:, 4] == min_area])[:, 0:3]
            stop_value = len(self.targets) - len(sub_area)
            # Part 3 - Explore Sub-Area with Minimum Distance Search
            while not len(self.targets) == stop_value:
                # Sub-Part 1 - Get Distance of Nodes From Drone and Store and Find Minimum One
                temp_targets = np.concatenate((sub_area,self.distance_from_targets(sub_area)), axis=1)
                check_target = temp_targets[np.argmin(temp_targets[:, 3]), :]
                # Sub-Part 2 - Send Drone to Minimum One
                checked_targets = self.drone_check_target(check_target, sub_area)
                # Sub-Part 3 - Delete nodes from sub_area too
                sub_area = self.delete_checked_targets(checked_targets, sub_area, 1)
        #####################################################################################
        stop = time()

        if self.debug:
            print str('Time to check all Targets by Drone was ') + str(stop - start) + str(' seconds.')

        return self.evaluation_result

    # Distance from Drone to Targets Calculation Function
    def distance_from_targets(self, potential_targets):
        # Get Position of Drone
        tf = self.pid.tf.lookup_transform(self.pid.drone_base_link, self.pid.map_frame,
                                                 rospy.Time(0), rospy.Duration(1.0))
        drone_x = tf.transform.translation.x
        drone_y = tf.transform.translation.y
        distance_array = np.zeros((potential_targets.shape[0], 2), dtype=np.float32)
        # Calculate Distance of Potential Targets to Drone
        count = 0
        for target in potential_targets:
            distance_array[count, 0] = hypot(drone_x - target[1],drone_y - target[2])

            if target[1]*target[2] < 0:
                if target[1] < 0:
                    distance_array[count, 1] = 1
                else:
                    distance_array[count, 1] = 3
            elif target[1] == 0:
                if target[2] > 0:
                    distance_array[count, 1] = 2
                else:
                    distance_array[count, 1] = 3
            elif target[2] == 0:
                if target[1] > 0:
                    distance_array[count, 1] = 3
                else:
                    distance_array[count, 1] = 2
            else:
                if target[1] > 0:
                    distance_array[count, 1] = 2
                else:
                    distance_array[count, 1] = 4
            count += 1
        # Return Result
        return distance_array

    # Color Evaluation of Each Target Function
    def drone_check_target(self, check_target, all_targets):

        # Initialize Checked Nodes Array
        checked_targets = np.array([-1], dtype=np.int64)
        # Use Plane PID to Reach Desired Target
        while not self.pid.sent_drone_to_position(check_target[1], check_target[2], 0):
            pass
        # Take Image and TF of Drone using Camera Callback Function and a Flag
        self.take_image_and_tf_flag = True
        while self.take_image_and_tf_flag:
            pass
        # Get Pixel Position of Target Nodes in Drone Image Taken
        targets_pixel = self.pixel_transform.transform_to_pixel(all_targets, self.transform , self.pid.map_frame)
        (rows, cols, _) = self.drone_image.shape
        # Run Loop For All Targets
        for target_pixel in targets_pixel:
            # Check if Target is inside the Image
            bounders = self.pixel_transform.target_limits_in_image(rows, cols, target_pixel[1], target_pixel[2])
            # If Inside Color Evaluate the Target
            if bounders != -1:
                # Free Space Calculation
                self.evaluation_result.color_weight[int(target_pixel[0])] =\
                    free_space_calculation(np.asarray(self.drone_image, dtype=np.float64),
                                           rows, cols, np.asarray(bounders, dtype=np.int32), 0.0)
                # Corners In Image Calculation
                self.evaluation_result.corner_number[int(target_pixel[0])] =\
                    self.corner_detection.corner_detection(self.corner_image)
                # Add to Checked Targets
                checked_targets = np.vstack([checked_targets, int(target_pixel[0])])
        # After Targets Checked, delete all the Checked Nodes
        self.delete_checked_targets(checked_targets)
        # Return Checked Targets Array
        return checked_targets

    # Delete Target from Main Array Function
    def delete_checked_targets(self, target_to_check, delete_array = [], mode = 0):

        counter = 0
        # Delete From Main Array
        for n in self.targets:
            if n[0] in target_to_check[:, 0]:
                self.targets = np.delete(self.targets, counter, axis=0)
                counter -= 1
            counter += 1

        # If mode == 1 :: Delete from Sub-Area too
        if mode == 1:
            counter = 0
            for n in delete_array:
                if n[0] in target_to_check[:, 0]:
                    delete_array = np.delete(delete_array, counter, axis=0)
                    counter -= 1
                counter += 1
            return delete_array
        else:
            return True

    # Callback Function for Drone Bottom Camera Topic that Saves Image and TF at the Right Time
    def drone_camera_cb(self, image):

        if self.take_image_and_tf_flag:
            # Save Current Transform from Camera to Map and Image
            self.transform = self.pid.tf.lookup_transform(self.drone_camera_frame,self.pid.map_frame,
                                                          rospy.Time(0),rospy.Duration(1))
            try:
                self.drone_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
                self.corner_image = image
            except CvBridgeError as e:
                print(e)
            # Convert Image to Black and Red Only
            img_hsv = cvtColor(self.drone_image, COLOR_BGR2HSV)
            mask = inRange(img_hsv, np.array([0, 50, 50]), np.array([10, 255, 255])) \
                   + inRange(img_hsv, np.array([170, 50, 50]), np.array([180, 255, 255]))
            self.drone_image[np.where(mask == 0)] = 0
            # Reset the Flag to Continue
            self.take_image_and_tf_flag = False


@jit(nopython=True)
def free_space_calculation(image, rows, cols, bounders, counted):
    for y in range(rows):
        for x in range(cols):
            if image[y][x][2] > 100:
                if bounders[0] < x < bounders[1] and bounders[4] < y < bounders[5]:
                    weight = 1.0
                elif bounders[2] < x < bounders[3] and bounders[6] < y < bounders[7]:
                    weight = 0.5
                else:
                    weight = 0.25
                counted += weight * 1.0
    return counted
