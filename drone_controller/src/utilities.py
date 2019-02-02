#!/usr/bin/env python

import numpy as np
from rospy import Time, Timer, Duration, get_param, get_time, Publisher, Subscriber
from math import fabs, pi
from time import time
from ar_pose.msg import ARMarker
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion, quaternion_matrix

# OpenCV Libraries Added for Corner_Detection Class
from cv_bridge import CvBridge, CvBridgeError
# cv2 Functions
from cv2 import inRange, cvtColor, cornerHarris, COLOR_BGR2HSV, COLOR_BGR2GRAY
# cv Functions
import cv2.cv as cv


# Plane PID Class
class Plane_PID:

    # Constructor
    def __init__(self):

        # TF Listener
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf)

        # Flags
        self.initialize = False

        # Parameters
        self.last_t = 0.0
        self.lin_i_x = 0.0
        self.lin_i_y = 0.0
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.error = 0.15
        self.max_speed = 1.8
        self.velocity = Twist()
        self.empty = Twist()

        # FIXME: Parameter from YAML File
        self.drone_base_link = get_param('drone_base_link')
        self.map_frame = get_param('map_frame')
        self.drone_speed_topic = get_param('drone_speed_topic')

        # Publish Speed Topic
        self.cmd_topic = Publisher(self.drone_speed_topic, Twist, queue_size=10)

    # Initialize PID Timer Function
    def initialize_pid(self):

        self.last_t = time()
        self.lin_i_x = 0.0
        self.lin_i_y = 0.0

    # Initialize Gains Function
    def initialize_gains(self,kp=0.3, ki=0, kd=0, error=0.2, speed=1.8):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = error
        self.max_speed = speed
        self.initialize = True

    # PID Function for X-Y Plane
    def xy_pid(self, x, x_last, y, y_last):

        if self.initialize:
            # Initialize
            kp_1 = ki_1 = kd_1 = 0.0  # For X-Control
            kp_2 = ki_2 = kd_2 = 0.0  # For Y-Control
            # Get current dt
            dt = (time() - self.last_t)
            self.last_t = time()
            if dt > 0.1:
                self.lin_i_x = self.lin_i_y = 0.0
            # Integral
            self.lin_i_x += x * dt
            self.lin_i_y += y * dt
            # Derivative
            lin_d_x = (x - x_last) / dt
            lin_d_y = (y - y_last) / dt
            # Choose Gain
            if fabs(x) > 0.1 or fabs(y) > 0.1:
                if fabs(x) > 0.1:
                    kp_1 = self.kp
                    ki_1 = self.ki
                    kd_1 = self.kd
                if fabs(y) > 0.1:
                    kp_2 = self.kp
                    ki_2 = self.ki
                    kd_2 = self.kd

            self.velocity.linear.x = kp_1 * x + ki_1 * self.lin_i_x + kd_1 * lin_d_x
            self.velocity.linear.y = kp_2 * y + ki_2 * self.lin_i_y + kd_2 * lin_d_y
            # Fixing Limits in Speed Assigned to [-MAX_Speed, MAX_Speed]
            self.velocity.linear.x = np.clip(self.velocity.linear.x, -self.max_speed, self.max_speed)
            self.velocity.linear.y = np.clip(self.velocity.linear.y, -self.max_speed, self.max_speed)

            return self.velocity

    # Position Error in Map Function
    def pose_error(self, x, y, z):

        target_to_map = PoseStamped()
        target_to_map.pose.position.x = x
        target_to_map.pose.position.y = y
        target_to_map.pose.position.z = z

        transform = self.tf.lookup_transform(self.drone_base_link, self.map_frame, Time(0), Duration(1.0))

        error = do_transform_pose(target_to_map, transform)

        return error.pose.position.x,error.pose.position.y, error.pose.position.z

    # Sent Drone to Position Function
    def sent_drone_to_position(self, x, y, z, kp=0.3, ki=0.0, kd=0.0, error=0.2, speed=1.8):

        # Initialize Gains if they are Not Initialized
        if not self.initialize:
            self.initialize_gains(kp, ki, kd, error, speed)

        # Initialize Values
        first_run = True
        x_last = y_last = 0.0
        (error_x, error_y, _) = self.pose_error(x, y, z)
        if fabs(error_x) < self.error:
            error_x = 0.0
        if fabs(error_y) < self.error:
            error_y = 0.0

        # Run Loop Until You Reach The Target
        while not error_x == 0.0 or not error_y == 0.0:

            # Get Error_X and Error_Y
            (error_x, error_y, _) = self.pose_error(x, y, z)

            if fabs(error_x) < self.error:
                error_x = 0.0
            if fabs(error_y) < self.error:
                error_y = 0.0

            # Run PID
            if first_run:
                self.initialize_pid()
            speed = self.xy_pid(error_x, x_last, error_y, y_last)
            # Publish Speed
            if not first_run:
                time = get_time()
                while get_time() < time + 0.1:
                    self.cmd_topic.publish(speed)
            else:
                first_run = False

            self.cmd_topic.publish(self.empty)
            # Save last position for next PID run
            x_last = error_x
            y_last = error_y

        # Return True of Point is Reached
        return True


# Pose Follower Class
class Marker_Follower:

    # Constructor
    def __init__(self):

        # TF Listener
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf)

        # Flags
        self.initialize = False
        self.marker_detected = False    # True if Drone Camera has detected a Tag
        self.recovery_behavior = False  # Used in recovery() Function if Marker is Lost

        # PID Parameters
        self.last_t = 0.0
        self.lin_i_x = 0.0
        self.lin_i_y = 0.0
        self.lin_i_z = 0.0
        self.rot_i_z = 0.0

        # Gains ( with Initial Values)
        self.kp = 0.3
        self.ki = 0.01
        self.kd = 0.06
        self.r_kp = 3.0
        self.r_ki = 1.5
        self.r_kd = 0.57
        self.r_kp_s = 0.6
        self.r_ki_s = 0.13
        self.r_kd_s = 0.07
        self.error = 0.25
        self.max_speed = 1.0
        self.max_z_speed = 1.5

        # Parameter from YAML File
        self.drone_base_link = get_param('drone_base_link')
        self.drone_base_footprint = get_param('drone_base_footprint')
        self.map_frame = get_param('map_frame')
        self.ar_pose_frame = get_param('ar_pose_frame')
        self.drone_camera_frame = get_param('drone_camera_frame')
        self.drone_speed_topic = get_param('drone_speed_topic')
        self.ar_pose_topic = get_param('ar_pose_topic')
        self.time_to_considered_marker_lost = get_param('time_to_considered_marker_lost')
        # Parameters
        self.last_time_marker_seen = 0.0    # Hold the time that Tag last seen, for Setting Marker Detected flag ON/OFF
        # Ar Marker Topic Subscriber
        self.marker_subscriber = Subscriber('/ar_pose_marker', ARMarker, self.marker_cb)
        # Timer for Checking if Marker is Lost
        self.check_marker_timer = Timer(Duration(0.05), self.check_marker_existence)

    # PID Function
    def pose_pid(self, x, x_last, y, y_last, z, z_last, yaw, yaw_last):

        # Initialize
        kp_1 = ki_1 = kd_1 = 0.0    # For X-Control
        kp_2 = ki_2 = kd_2 = 0.0    # For Y-Control
        kp_3 = ki_3 = kd_3 = 0.0    # For Z-Control
        kp_4 = ki_4 = kd_4 = 0.0    # For Yaw-Control

        # Get current dt
        dt = (time() - self.last_t)
        self.last_t = time()
        # Prevent High Fluctuations of Integral Term
        if dt > 0.1:
            self.lin_i_x = self.lin_i_y = self.lin_i_z = self.rot_i_z = 0.0

        # Integral Term
        self.lin_i_x += x*dt
        self.lin_i_y += y*dt
        self.lin_i_z += z*dt
        self.rot_i_z += yaw*dt
        # Derivative Term
        lin_d_x = (x - x_last)/dt
        lin_d_y = (y - y_last)/dt
        lin_d_z = (z - z_last)/dt
        rot_d_z = (yaw - yaw_last)/dt

        # Choose Gain
        # For X-Y : 0.3,0.01,0.06 ||| For Z: 0.3,0.01,0.06 ||| For Yaw: 3,1.5,0.57
        if fabs(x) > 0.1 or fabs(y) > 0.1:
            if fabs(x) > 0.1:
                kp_1 = 0.3
                ki_1 = 0.01
                kd_1 = 0.06
            if fabs(y) > 0.1:
                kp_2 = 0.3
                ki_2 = 0.01
                kd_2 = 0.06

        if fabs(np.rad2deg(yaw)) > 1.0:
            # Rotate Full Speed
            if fabs(x) < 0.2 and fabs(y) < 0.2:
                kp_4 = 3
                ki_4 = 1.5
                kd_4 = 0.57
            # Rotate Carefully!
            else:
                kp_4 = 0.6
                ki_4 = 0.13
                kd_4 = 0.07

        if fabs(np.rad2deg(yaw)) < 1.0 and fabs(x) < 0.1 and fabs(y) < 0.1:
            kp_3 = 0.3
            ki_3 = 0.01
            kd_3 = 0.06

        velocity = Twist()

        # Calculate Speed
        velocity.linear.x = kp_1*x + ki_1*self.lin_i_x + kd_1*lin_d_x
        velocity.linear.y = kp_2*y + ki_2*self.lin_i_y + kd_2*lin_d_y
        velocity.linear.z = kp_3*z + ki_3*self.lin_i_z + kd_3*lin_d_z
        velocity.angular.z = -(kp_4*yaw + ki_4*self.rot_i_z + kd_4*rot_d_z)

        # Fixing Limits in Speed Assigned to [-MAX,MAX]
        velocity.linear.x = np.clip(velocity.linear.x, -self.max_speed, self.max_speed)
        velocity.linear.y = np.clip(velocity.linear.y, -self.max_speed, self.max_speed)
        velocity.linear.z = np.clip(velocity.linear.z, -self.max_z_speed, self.max_z_speed)
        velocity.angular.z = np.clip(velocity.angular.z, -self.max_speed, self.max_speed)

        return velocity

    # Initialize PID Timer Function
    def initialize_pid(self):

        self.last_t = time()
        self.lin_i_x = 0.0
        self.lin_i_y = 0.0
        self.lin_i_z = 0.0
        self.rot_i_z = 0.0

    # Initialize Gains Function
    def initialize_gains(self, kp=0.3 , ki=0.01, kd=0.06, r_kp=3.0, r_ki=1.5, r_kd=0.57, r_kp_s=0.6,
                         r_ki_s=0.13, r_kd_s=0.07, error=0.25, speed=1.0, z_speed=1.5):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.r_kp = r_kp
        self.r_ki = r_ki
        self.r_kd = r_kd
        self.r_kp_s = r_kp_s
        self.r_ki_s = r_ki_s
        self.r_kd_s = r_kd_s
        self.error = error
        self.max_speed = speed
        self.max_z_speed = z_speed

        self.initialize = True

    # Calculate Pose Error From Tag Function that Returns (x,y,z,yaw,height)
    def calculate_error_from_tag(self):

        # Calculate_TF and get Pose Error
        if self.can_transform() and self.marker_detected:

            # Get the most recent TF's
            tf_ = self.tf.lookup_transform(self.ar_pose_frame, self.drone_camera_frame, Time(0), Duration(1.0))
            h_tf = self.tf.lookup_transform(self.drone_base_footprint, self.drone_base_link, Time(0), Duration(1.0))
            # The Height that Drone is currently fly
            height = h_tf.transform.translation.z
            # Calculate Yaw from TF
            (_,_,yaw) = euler_from_quaternion([tf_.transform.rotation.x,tf_.transform.rotation.y,
                                               tf_.transform.rotation.z,tf_.transform.rotation.w])
            yaw += pi
            # Manipulate Yaw to Rotate Drone in the Right Direction
            if np.rad2deg(yaw) > 180:
                yaw -= 2*pi
            # Get the Rotation Matrix
            (r1, r2, r3,_) = quaternion_matrix([tf_.transform.rotation.x,tf_.transform.rotation.y,
                                                tf_.transform.rotation.z,tf_.transform.rotation.w])
            rotation_mat = np.asmatrix([r1[0:3],r2[0:3],r3[0:3]])
            # Get Position in Matrix Form
            pose_mat = np.asmatrix([[tf_.transform.translation.x],[tf_.transform.translation.y],[tf_.transform.translation.z]])
            # Calculate the Cartesian Error
            error = rotation_mat*pose_mat

            return [error.item(0),error.item(1),error.item(2),yaw,height]
        else:
            return [-1, -1, -1, -1, -1]

    # Callback Function for Ar Marker Topic that Saves Image and TF at the Right Time
    def marker_cb(self, marker):
        self.last_time_marker_seen = get_time()

    # Function that Check if Marker is Detected Called from Timer
    def check_marker_existence(self, event):
        if get_time() < self.last_time_marker_seen + self.time_to_considered_marker_lost:
            self.marker_detected = True
        else:
            self.marker_detected = False

    # Check for TF Between Drones Camera and Tag Availability Function
    def can_transform(self):
        if self.tf.can_transform(self.ar_pose_frame, self.drone_camera_frame, Time(0), Duration(1.0)):
            return True
        else:
            return False

    # Check if Target is Reached Function
    def check_target_reached(self):

        error = self.calculate_pose_error()

        if error != [-1, -1, -1, -1, -1]:
            if error[0] < 0.075 and error[1] < 0.05 and error[4] < 1.7 and np.rad2deg(error[3]) < 0.08:
                return True
            else:
                return False
        else:
            return False

    # Drone Marker Retrieval Function
    def drone_marker_retrieval(self):

        # Create Temporary Speeds
        speed = Twist()
        # Switching Between Rotate and Going Up using Recovery Behavior Flag
        if self.recovery_behavior:
            self.recovery_behavior = False
            speed.angular.z = 0.5
        if not self.recovery_behavior:
            self.recovery_behavior = True
            speed.linear.z = 0.5

        return speed


# Corner Detection Class
class Corner_Detection:

    # Constructor
    def __init__(self):

        # CvBridge Object
        self.bridge = CvBridge()

    # Detect Corners Function in Image
    def corner_detection(self, image):

        # Read the Image
        image_cp = self.bridge.imgmsg_to_cv2(image, "bgr8")
        # Convert it in Black and White
        m_image = cvtColor(image_cp, COLOR_BGR2HSV)
        mask = inRange(m_image, np.array([0, 100, 100]), np.array([10, 255, 255]))\
               + inRange(m_image, np.array([170, 50, 50]), np.array([180, 255, 255]))
        image_cp[np.where(mask == 0)] = [255, 255, 255]
        image_cp[np.where(mask == 255)] = 0
        # Get Image Ready for Harris Corner Algorithm
        gray = np.float32(cvtColor(image_cp, COLOR_BGR2GRAY))
        # Run Corner Detection and Find How Many Corners Exist in the Image
        corners = self.check_valid_corners(cornerHarris(gray, 2, 5, 0.04))
        # Return the Number of Corner Found
        return corners.shape[0]

    # Check for Valid Corners Detected Function
    @staticmethod
    def check_valid_corners(corners):

        # Get Limits of Corner Array (Image)
        lim_x = corners.shape[0]
        lim_y = corners.shape[1]
        # Get X-Y of Corners into intex_array!
        corners = np.asarray(np.nonzero(corners > 0.1*corners.max()), dtype=np.int32)
        corners = np.concatenate(((corners[:][0]).reshape(-1, 1),
                                  (corners[:][1]).reshape(-1, 1)), axis=1)
        # Delete Invalid Corners!
        # Part A - Delete Corners in the Limits of the Image!
        count = 0
        delete_array = []
        for n in corners:
            if n[0] < 5 or n[0] > lim_x - 5 or n[1] < 5 or n[1] > lim_y - 5:
                delete_array = np.insert(delete_array, 0, count)
            count += 1
        corners = np.delete(corners, delete_array, axis=0)
        # Part B - Delete Duplicate Corners in the Image!
        if corners.shape[0] > 1:

            corner_diff = np.concatenate((np.diff(corners[:, 0]).reshape(-1, 1),
                                          np.diff(corners[:, 1]).reshape(-1, 1)), axis=1)
            count = 0
            delete_array = []
            for p in corner_diff:
                if abs(p[0]) <= 20 and abs(p[1]) <= 20:
                    delete_array = np.insert(delete_array, 0, count + 1)
                count += 1
            corners = np.delete(corners, delete_array, axis=0)
        # Return the Final Corner Array
        return corners


# Point to Pixel Class
class Point_to_Pixel:

    # Transforms Points From Map to Pixel in Camera Image Function
    @staticmethod
    def transform_to_pixel(targets, transform, map_frame):

        # Initial Bottom Camera Parameter for Quadcopter AR Drone 2.0
        cam_mat = cv.fromarray(np.array([[374.670607, 0.0, 320.5],[0, 374.670607, 180.5],
                                         [0, 0, 1]], dtype=np.float32))

        r_v = t_v = cv.fromarray(np.float32([[0, 0, 0]]).reshape(-1, 3))

        coefficients = cv.fromarray(np.array([[0.0182389759532889],[0.0520276742502367],
                                              [0.00651075732801101],[0.000183496184521575],
                                              [0]], dtype=np.float32))
        # Create Position to Camera Array
        position = np.zeros((targets.shape[0], 3), dtype=np.float64)
        image_point = cv.fromarray(np.zeros((targets.shape[0], 2), dtype=np.float64).reshape(-1, 2))
        # Create PoseStamped() and TransformStamped() from given x,y,z!
        pose = PoseStamped()
        pose.header.frame_id = map_frame
        # Convert all Targets to Camera Frame
        count = 0
        for n in targets:
            pose.pose.position.x = n[1]
            pose.pose.position.y = n[2]
            pose.pose.position.z = 0
            pose = do_transform_pose(pose, transform)
            position[count] = np.float32([(pose.pose.position.x, pose.pose.position.y,
                                           pose.pose.position.z)]).reshape(-1, 3)
            count += 1
        # Use ProjectPoints2() to get Pixel Position
        cv.ProjectPoints2(cv.fromarray(position), r_v, t_v, cam_mat, coefficients, image_point)
        # Return Result
        return np.concatenate((targets[:, 0].reshape(-1, 1),
                               np.asarray(np.ceil(np.asarray(image_point[:, :], dtype=np.float64)),dtype=np.int64)), axis=1)

    # Calculating the Limits of Target inside Drone Image Function
    @staticmethod
    def target_limits_in_image(rows, cols, pixel_x, pixel_y):

        if pixel_x > cols - 59 or pixel_y > rows - 59 or pixel_x < 59 or pixel_y < 59:
            return -1

        else:
            # 1st Limits :
            low_x = pixel_x - 100
            up_x = pixel_x + 100
            low_y = pixel_y - 100
            up_y = pixel_y + 100

            # 2st Limits :
            if pixel_x - 200 >= 0:
                low_x2 = pixel_x - 200
            else:
                low_x2 = 0

            if pixel_x + 200 <= cols:
                up_x2 = pixel_x + 200
            else:
                up_x2 = cols

            if pixel_y - 200 >= 0:
                low_y2 = pixel_y - 200
            else:
                low_y2 = 0

            if pixel_y + 200 <= rows:
                up_y2 = pixel_y + 200
            else:
                up_y2 = rows

            return [low_x, up_x, low_x2, up_x2, low_y, up_y, low_y2, up_y2]