#!/usr/bin/env python

import rospy
import numpy as np
import threading
import imutils
import tf

from math import sqrt, pow, fabs
from time import time, sleep
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from skimage import exposure
from numba import jit
from _cpp_functions import ffi, lib
from skimage.morphology import disk
from scipy.spatial import distance
# TF for DroneCommunication Class
from tf2_ros import Buffer, TransformListener
from tf.transformations import euler_from_quaternion, quaternion_matrix
# Open CV Functiona Needed
from cv2 import COLOR_BGR2GRAY, bitwise_not, imshow, waitKey, destroyAllWindows, rectangle, cvtColor, cornerHarris, TM_CCOEFF_NORMED, matchTemplate, dilate, minMaxLoc
# Costume Services and Messages Added
from drone_controller.srv import ControlDrone


class OgmOperations:

    # Function that Clear OGM Map for Discontinuations of Laser Sensor
    @staticmethod
    def blurUnoccupiedOgm(ogm, ogml):

        return ogm_blur(ogm, ogml['min_x'], ogml['max_x'], ogml['min_y'], ogml['max_y'])

    # Function that Calculate OGM Useful Boundaries
    @staticmethod
    def findUsefulBoundaries(ogm, origin, resolution, print_result=True, step=20):

        min_x = origin['x'] / resolution
        min_y = origin['y'] / resolution
        max_x = origin['x'] / resolution
        max_y = origin['y'] / resolution
      
        x = ogm.shape[0]
        y = ogm.shape[1]

        # Search by x min
        ok = False
        for i in range(0, x, step):
            for j in range(0, y):
                if ogm[i][j] != 51:
                    min_x = i - step
                    ok = True
                    break
            if ok:
                break

        # Search by x max
        ok = False
        for i in range(min_x + 20, x, step):
            for j in range(0, y):
                if ogm[i][j] != 51:
                    ok = True
                    break
            if not ok:
                max_x = i
                break
            ok = False

        # Search by y min
        ok = False
        for j in range(0, y, step):
            for i in range(0, x):
                if ogm[i][j] != 51:
                    min_y = j - step
                    ok = True
                    break
            if ok:
                break

        # Search by y max
        ok = False
        for j in range(min_y + 20, y, step):
            for i in range(0, x):
                if ogm[i][j] != 51:
                    ok = True
                    break
            if not ok:
                max_y = j
                break
            ok = False

        if print_result:
            RvizHandler.printMarker([[min_x * resolution + origin['x'], min_y * resolution + origin['y']],
                                     [max_x * resolution + origin['x'], min_y * resolution + origin['y']],
                                     [max_x * resolution + origin['x'], max_y * resolution + origin['y']],
                                     [min_x * resolution + origin['x'], max_y * resolution + origin['y']]],
                                    1, 0, "map", "art_ogm_boundary", [0, 0.9, 0, 1.0], 0.2)

        return {'min_x': min_x, 'max_x': max_x, 'min_y': min_y, 'max_y': max_y}

    # Function that Calculate OGM Limits Point to be Explored by Drone
    @staticmethod
    def findLimitPoints(ogm, origin,  resolution, find_step=20, search_step=10,
                        max_limits=10, map_size=[], debug=False, visualize=True):

        ogm_limit = OgmOperations.findUsefulBoundaries(ogm, origin, resolution, step=find_step)

        # Search inside Useful Boundaries
        x_min = ogm_limit['min_x'] + 20
        x_max = ogm_limit['max_x']
        y_min = ogm_limit['min_y'] + 20
        y_max = ogm_limit['max_y']

        # Lists holding Limit Points
        mim_x_data = []
        min_y_data = []
        max_x_data = []
        max_y_data = []

        # Check if Step is Bigger that OGM Limits and Return Error
        if search_step > (x_max - x_min) or search_step > (y_max - y_min):
            return None
        # Dynamically Change Search Step to Keep Number of Limit Points Constant
        else:
            while 2*((x_max - x_min)//search_step + (y_max - y_min)//search_step) > max_limits:
                search_step += 10
            # Minimum Acceptable Distance Between Min-Max in Search
            min_distance_x = 0.2*(x_max - x_min)
            min_distance_y = 0.2*(y_max - y_min)

            if debug:
                print str('Find Limit Points: Minimum Distance in X-Axis is ') + str(min_distance_x) \
                      + str(' and minimum Distance in Y-Axis is ') + str(min_distance_y)
                print str('Find Limit Points :OGM Limits: [') + str(x_min) + str(',') + str(x_max) + str('], [') \
                      + str(y_min) + str(', ') + str(y_max) + str(']')

        step = search_step
        ###############################################
        # Search in X-Axis
        for i in range(x_min, x_max, step):
            for j in range(y_min, y_max):
                # Find Where OGM starts to be Discovered
                if ogm[i][j] != 51:
                    mim_x_data.append([i, j])
                    break
        minimum_x = np.array(mim_x_data).reshape(-1,2)

        ok = False
        for n in range(0, len(minimum_x)):
            (x_low, y_low) = minimum_x[n, :]
            for j in range(y_low + 1, y_max):
                # Find Where OGM start to be Undiscovered
                if ogm[x_low][j] == 51:
                    temp = j
                    ok = True
                    # Check the Rest of the Line!
                    for k in range(temp, y_max):
                        if ogm[x_low][k] != 51:
                            ok = False
                            break
                    # Store or Continue Search!
                    if ok:
                        if abs(temp - x_low) > min_distance_x:
                            max_x_data.append([x_low, temp])
                        break
                    else:
                        continue
        maximum_x = np.array(max_x_data).reshape(-1,2)

        ###############################################
        # Search in Y-Axis
        for j in range(y_min, y_max, step):
            for i in range(x_min, x_max):
                # Find Where OGM starts to be Discovered
                if ogm[i][j] != 51:
                    min_y_data.append([i, j])
                    break
        minimum_y = np.array(min_y_data).reshape(-1,2)

        ok = False
        for n in range(0, len(minimum_y)):
            (x_low, y_low) = minimum_y[n, :]
            for i in range(x_low+1, x_max):
                # Find Where OGM start to be Undiscovered
                if ogm[i][y_low] == 51:
                    temp = i
                    ok = True
                    # Check the Rest of the Line!
                    for k in range(temp, x_max):
                        if ogm[k][y_low] != 51:
                            ok = False
                            break
                    # Store or Continue Search!
                    if ok:
                        if abs(temp - y_low) > min_distance_y:
                            max_y_data.append([temp, y_low])
                        break
                    else:
                        continue
        maximum_y = np.array(max_y_data).reshape(-1,2)

        ###############################################
        # Keep Limit Points :  (1) Distance Bigger that 5 Meters,
        #                      (2) Unique,
        #                      (3) Not Map Limit Points
        limits = OgmOperations.unique(np.array(np.concatenate((minimum_x, minimum_y, maximum_x, maximum_y), axis=0), dtype=np.float64))
        limits = OgmOperations.near_map_limits(limits, [map_size[0][0], map_size[3][0],map_size[0][1],map_size[3][1]], close=20)
        if len(limits) > 1:
            limits = OgmOperations.delete_close_limit_points(limits, resolution, origin, erase_distance=5)

         # Visualize in RViz
        if visualize and len(limits) != 0:
            viz_limits = np.array(limits, dtype=np.float64)
            for n in viz_limits:
                n[0] = n[0]*resolution + origin['x']
                n[1] = n[1]*resolution + origin['y']
            RvizHandler.printMarker(viz_limits, 1, 0, "map", "art_ogm_limit_points", [0.7, 0.3, 0, 1.0], 0.2)

        return limits, ogm_limit

    # Function that Delete Duplicate Points Calculated in findLimitPoints (Pixel Expressed)
    @staticmethod
    def unique(limits):

        # Calculate Distance of Limits Points
        dist = distance.cdist(limits, limits, 'euclidean')
        l = np.argwhere(dist == 0)
        first = True
        delete_array = [[0, 0]]
        # Find Duplicate Points
        for x, y in l:
            if x != y:
                if first:
                    first = False
                    delete_array = np.append(delete_array, [[x, y]], axis=0)
                    delete_array = np.delete(delete_array, [0], axis=0)
                else:
                    if not ([y, x] in delete_array):
                        delete_array = np.append(delete_array, [[x, y]], axis=0)
        # Delete Duplicate Points (If Necessary)
        if first:
            return limits
        else:
            return np.delete(limits, delete_array[:, 0], axis=0)

    @staticmethod
    def near_map_limits(limits, map_limits, close=20):

        index = []
        # Check If Limits Found are close to Map Limits and Delete them
        for idx, n in enumerate(limits):
            if fabs(map_limits[0] - n[0]) <= close or fabs(map_limits[1] - n[0]) <= close:
                index.append(idx)
            elif fabs(map_limits[2] - n[1]) <= close or fabs(map_limits[3] - n[1]) <= close:
                index.append(idx)

        if len(index) != 0:
            return np.delete(limits, index, axis=0)
        else:
            return limits

    @staticmethod
    def delete_close_limit_points(limits, resolution, origin, erase_distance=5):

        # Convert them to Map Points and in List Form
        for n in limits:
            n[0] = n[0] * resolution + origin['x']
            n[1] = n[1] * resolution + origin['y']
        limits = limits.tolist()

        # Delete Near Limits Selected
        change = True
        while change:
            change = False
            for i in range(0, len(limits)):
                for j in range(0, len(limits)):
                    # Avoid checking Nodes Distance to Itself
                    if i == j:
                        continue
                    n1 = limits[i]
                    n2 = limits[j]
                    # Check Distance
                    if sqrt(pow(n1[0] - n2[0], 2) + pow(n1[1] - n2[1], 2)) < erase_distance:
                        change = True
                        del limits[i]
                        # Break from j Loop
                        break
                # Break from i Loop
                if change:
                    break

        for n in limits:
            n[0] = (n[0] - origin['x']) // resolution
            n[1] = (n[1] - origin['y']) // resolution

        return np.asarray(limits, dtype=np.int64)

class ImageMatching:

    # Function that Performs Grayscale Template Matching Invariant to Rotation and Scale
    @staticmethod
    def template_matching(ogm, drone_image, lim_x, lim_y, drone_yaw=0, window=300,
                          s_threshold=0.30, debug=False, print_result=False):

        #########################################################
        # Part 1 - Convert to Compatable Grayscale
        ogm_cp = bitwise_not(exposure.rescale_intensity(np.array(ogm, dtype=np.uint8),
                                                        in_range=(0, 100), out_range=(0, 255)))
        rotated_ogm = np.array(imutils.rotate(ogm_cp, angle=drone_yaw),
                           dtype=np.uint8)[lim_x - window // 2:lim_x + window // 2, lim_y - window:lim_y + window]
        drone_cp = np.array(np.array(drone_image[:, :, 1]), np.uint8)

        #########################################################
        # Part 2 - Run the Matching Algorithm
        res = ImageMatching.scale_free_template_match(rotated_ogm, drone_cp)

        #########################################################
        # Part 3 - Reconstruct the OGM with Added Data
        if res is not -1:
            (min, max, similarity) = res

            if debug:
                print str('Images Similarity is ') + str(similarity)

            if similarity > s_threshold:
                # Resize Drone Image to Match Scale of Matching
                drone_cp = imutils.resize(drone_cp, width=(max[0] - min[0]), height=(max[1] - min[1]))
                # Add Data to Rotated OGM and Rotate Back to 0 Degrees
                f_ogm = imutils.rotate(ogm_reconstruct(rotated_ogm, drone_cp,
                                                       rotated_ogm.shape[0], rotated_ogm.shape[1],
                                                       min[0], max[0],
                                                       min[1], max[1]), angle=-drone_yaw)
                # Final OGM Reconstruction
                (ogm, v_ogm) = ImageMatching.ogm_image_merge(ogm, f_ogm, [lim_x, lim_y], window)

                if print_result:
                    # Visualize the Result
                    imshow("Final OGM After Reconstruct", v_ogm)
                    waitKey(3000)
                    destroyAllWindows()
                    sleep(0.5)

        return ogm

    # Function that Performs Template Matching for Different Scale Factors of Source Image
    @staticmethod
    def scale_free_template_match(source, template, min=0.1, max=10.0,
                                  samples=10, debug=False):

        # Part 1 - Get Initial Width and Height && Calculate Minimum Scale
        (w_t, h_t) = template.shape[::-1]
        found = None

        # Part 2 - Loop over different Scales of Source Image
        for scale in np.linspace(min, max, samples)[::-1]:

            # Resize the Image According to the Scale, and keep track of Ratio
            resized = imutils.resize(source, width=int(source.shape[1]*scale))
            ratio = source.shape[1]/float(resized.shape[1])
            # If Resized is Bigger than the Source then Break from Loop
            if resized.shape[0] < h_t or resized.shape[1] < w_t:
                break
            # Matching to find the Template in the Image
            result = matchTemplate(resized, template, TM_CCOEFF_NORMED)
            (_, maxVal, _, maxLoc) = minMaxLoc(result)
            # Replace if we have found a new Maximum Correlation Value
            if found is None or maxVal > found[0]:
                found = (maxVal, maxLoc, ratio, scale)

        # Part 3 - Compute the Coordinate of the Bounding Box Using Ration (If Match Found)
        if found is not None:
            (maxVal, maxLoc, ratio, scale_factor) = found
            (x_min, y_min) = (int(maxLoc[0]*ratio), int(maxLoc[1]*ratio))
            (x_max, y_max) = (int((maxLoc[0] + w_t)*ratio), int((maxLoc[1] + h_t)*ratio))

            if debug:
                # Visualize the Result
                rectangle(source, (x_min, y_min), (x_max, y_max), 255, 2)
                imshow("Match with Scale Factor=%d" % scale_factor, source)
                waitKey(5000)
                destroyAllWindows()

            return [x_min, y_min], [x_max, y_max], maxVal
        else:
            return -1

    # Function Performing Merging of New Data to Source
    @staticmethod
    def ogm_image_merge(source, new_data, coordinates, window,
                        e_h=40, e_w=80, debug=False):

        # Keep Important Part of New Data
        new_data = np.array(new_data, dtype=np.uint8)[e_h:new_data.shape[0] - e_h, e_w:new_data.shape[1] - e_w]
        box = [coordinates[0] - window // 2 + e_h, coordinates[0] + window // 2 - e_h,
               coordinates[1] - window + e_w, coordinates[1] + window - e_w]
        # Copy New Data to Source
        for h in range(box[0], box[1]):
            for w in range(box[2], box[3]):
                # It's Free
                if new_data[h - box[0]][w - box[2]] > 230:
                    source[h][w] = 0
                # It's Occupied
                elif new_data[h - box[0]][w - box[2]] < 25:
                    source[h][w] = 100
        # Return According to Debug Flag
        if debug:
            viz_source = bitwise_not(exposure.rescale_intensity(np.array(source, dtype=np.uint8), in_range=(0, 100), out_range=(0, 255)))
            return source, viz_source
        else:
            return source, None

    # Function that Performs Image Matching with Known Image Position Correlation
    @staticmethod
    def ogm_match_with_known_image_pose(ogm, new_data, coordinates, map_boundries, debug=False):

        #########################################################
        # Part 1 - Reshape New Data to Grayscale
        new_data = np.array(np.array(new_data[:, :, 1]), np.uint8)
        if debug:
            viz_in_ogm = bitwise_not(exposure.rescale_intensity(np.array(ogm, dtype=np.uint8),
                                                                in_range=(0, 100), out_range=(0, 255)))
        #########################################################
        # Part 2 - Scale New Data
        # Hint : Scale Factor Known by Knowing Height in which Image is Taken and Camera Parameters
        '''
        new_width = 230 for 7 Meters Flight - Make it a Parameter!
        new_width = 150 for 5 Meters Flight - Make it a Parameter!
        '''
        # Hint : Must use Dilate to Make Obstacles Smaller
        new_data = imutils.resize(dilate(new_data, disk(2), iterations=2), width=140)
        (w_dr, h_dr) = new_data.shape[::-1]

        #########################################################
        # Part 3 - Reconstruct OGM
        # Hint : Must use Yaw and Map Limit Points !!!
        # Hint h_side, w_side must be Absolute Numbers
        '''
        coordinates[0] - Up/Down in Map (Height)
        coordinates[1] - Left/Right in Map (Width)
        coordinates[2] - Theta Angle
        map_boundries = [Min_W, Max_W, Min_H, Max_H]
        '''

        # TODO: Must Not Copy Data Outside OGM Limits of Map!
        ogm = ogm_kl_reconstruct(ogm=ogm, new_data=new_data, h_side=h_dr//2, w_side=w_dr//2,
                                 h_p=coordinates[0], w_p=coordinates[1],
                                 c_th=np.cos(np.deg2rad(coordinates[2])),
                                 s_th=np.sin(np.deg2rad(coordinates[2])),
                                 map_min_w=map_boundries[0], map_max_w=map_boundries[1],
                                 map_min_h=map_boundries[2], map_max_h=map_boundries[3])

        if debug:
            viz_ogm = bitwise_not(exposure.rescale_intensity(np.array(ogm, dtype=np.uint8),
                                                             in_range=(0, 100), out_range=(0, 255)))
            imshow('Initial OGM', viz_in_ogm)
            imshow('Drone Image', new_data)
            imshow('Result OGM', viz_ogm)
            waitKey(0)
            destroyAllWindows()

        return ogm


class CornerDetection:

    # Function that Detect Corners in Image (Mode 0/1 :: Color Image/Gray Image)
    @staticmethod
    def corner_detection(image, mode=0):
        # Mode 0 :: Color Image || Mode 1 :: Gray Image

        # Get Image Ready for Harris Corner Algorithm
        if mode == 1:
            gray = np.float32(image)
        else:
            gray = np.float32(cvtColor(image, COLOR_BGR2GRAY))
        # Run Corner Detection and Find How Many Corners Exist in the Image
        corners = ImageTools.check_valid_corners(cornerHarris(gray, 2, 5, 0.04))
        return corners.shape[0]

    # Function that Check if Corners Detected are Valid
    @staticmethod
    def check_valid_corners(corners):

        # Part 1 - Get Boundaries of Image
        limit_x = corners.shape[0]
        limit_y = corners.shape[1]
        # Part 2 - Get X-Y of Corners into intex_array!
        corners = np.asarray(np.nonzero(corners > 0.1*corners.max()), dtype=np.int32)
        corners = np.concatenate(((corners[:][0]).reshape(-1, 1), (corners[:][1]).reshape(-1, 1)), axis=1)
        # Part 3 - Delete Invalid Corners!
        # SubPart A - Delete Corners in the Limits of the Image!
        count = 0
        delete_array = []
        for n in corners:
            if n[0] < 5 or n[0] > limit_x - 5 or n[1] < 5 or n[1] > limit_y - 5:
                delete_array = np.insert(delete_array, 0, count)
            count += 1
        corners = np.delete(corners, delete_array, axis=0)
        # SubPart B - Delete Duplicate Corners in the Image!
        if corners.shape[0] > 1:
            corner_diff = np.concatenate((np.diff(corners[:, 0]).reshape(-1, 1), np.diff(corners[:, 1]).reshape(-1, 1)), axis=1)
            count = 0
            delete_array = []
            for p in corner_diff:
                if abs(p[0]) <= 20 and abs(p[1]) <= 20:
                    delete_array = np.insert(delete_array, 0, count + 1)
                count += 1
            corners = np.delete(corners, delete_array, axis=0)

        return corners


class Print:

    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    ORANGE = '\033[93m'
    RED = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

    @staticmethod
    def art_print(txt, color):
        print color + str(txt) + Print.END


class TimerThread:

    def __init__(self, timeout=10000):
        self.timeout = timeout
        self._expired = True
        self.timerThread = threading.Timer(self.timeout, self.timeout_reached)

    def start(self):
        self._expired = False
        self.timerThread = threading.Timer(self.timeout, self.timeout_reached)
        self.timerThread.start()

    def reset(self):
        self.timerThread.cancel()
        self._expired = False
        self.timerThread = threading.Timer(self.timeout, self.timeout_reached)
        self.timerThread.start()

    def stop(self):
        self.timerThread.cancel()

    def timeout_reached(self):
        self._expired = True

    @property
    def expired(self):
        return self._expired


class Cffi:

    @staticmethod
    def brushfireFromObstacles(ogm, brush, ogml):

        x = [np.array(v, dtype='int32') for v in ogm]
        xi = ffi.new(("int* [%d]") % (len(x)))
        for i in range(len(x)):
            xi[i] = ffi.cast("int *", x[i].ctypes.data)

        y = [np.array(v, dtype='int32') for v in brush]
        yi = ffi.new(("int* [%d]") % (len(y)))
        for i in range(len(y)):
            yi[i] = ffi.cast("int *", y[i].ctypes.data)

        lib.brushfireFromObstacles(xi, yi, len(x), len(x[0]), ogml['min_x'], ogml['max_x'], ogml['min_y'], ogml['max_y'])
        brush = cp_brush(brush, yi, brush.shape[0], brush.shape[1])

        return brush

    @staticmethod
    def thinning(skeleton, ogml):

        x = [np.array(v, dtype='int32') for v in skeleton]
        xi = ffi.new(("int* [%d]") % (len(x)))
        for i in range(len(x)):
            xi[i] = ffi.cast("int *", x[i].ctypes.data)

        y = [np.array(v, dtype='int32') for v in skeleton]
        yi = ffi.new(("int* [%d]") % (len(y)))
        for i in range(len(y)):
            yi[i] = ffi.cast("int *", y[i].ctypes.data)

        lib.thinning(xi, yi, len(x), len(x[0]), ogml['min_x'], ogml['max_x'], ogml['min_y'], ogml['max_y'])
        skeleton = cp_thinning(skeleton, yi, skeleton.shape[0], skeleton.shape[1])

        return skeleton

    @staticmethod
    def prune(skeleton, ogml, iterations):

        x = [np.array(v, dtype='int32') for v in skeleton]
        xi = ffi.new(("int* [%d]") % (len(x)))
        for i in range(len(x)):
            xi[i] = ffi.cast("int *", x[i].ctypes.data)

        y = [np.array(v, dtype='int32') for v in skeleton]
        yi = ffi.new(("int* [%d]") % (len(y)))
        for i in range(len(y)):
            yi[i] = ffi.cast("int *", y[i].ctypes.data)

        lib.prune(xi, yi, len(x), len(x[0]), ogml['min_x'], ogml['max_x'], ogml['min_y'], ogml['max_y'], iterations)
        skeleton = cp_prune(skeleton, yi, skeleton.shape[0], skeleton.shape[1])

        return skeleton


class RvizHandler:

    markers_publisher = rospy.Publisher('art_rviz_markers', MarkerArray, queue_size = 100)
    # Poses: list of coordinates
    # m_type: http://wiki.ros.org/rviz/DisplayTypes/Marker
    # action: 0 = add/modify, 1 = (deprecated), 2 = delete, New in Indigo 3 = deleteall
    # frame: Usually map
    # ns: whatever
    # color: [r,g,b,a]
    # scale: [sx,sy,sz]
    @staticmethod
    def printMarker(poses, m_type, action, frame, ns, color, scale):

        # Publish the targets for visualization purposes
        markers = MarkerArray()
        c = 0
        for s in poses:
            c += 1
            st = Marker()
            st.header.frame_id = frame
            st.ns = ns
            st.id = c
            st.header.stamp = rospy.Time(0)
            st.type = m_type
            st.action = action

            st.pose.position.x = s[0]
            st.pose.position.y = s[1]

            st.color.r = color[0]
            st.color.g = color[1]
            st.color.b = color[2]
            st.color.a = color[3]

            st.scale.x = scale
            st.scale.y = scale
            st.scale.z = scale
            markers.markers.append(st)

            if ns == 'art_topological_nodes':
                c += 1
                st = Marker()
                st.header.frame_id = frame
                st.ns = ns
                st.id = c
                st.header.stamp = rospy.Time(0)
                st.type = 9
                st.text = str((int) (c/2 - 1))
                st.action = action

                st.pose.position.x = s[0] + 0.15
                st.pose.position.y = s[1] + 0.15
                st.pose.position.z = 0.2

                st.color.r = 0
                st.color.g = 0
                st.color.b = 0
                st.color.a = 0.8

                st.scale.x = scale * 4
                st.scale.y = scale * 4
                st.scale.z = scale * 4
                markers.markers.append(st)

        RvizHandler.markers_publisher.publish(markers)


class DroneCommunication:

    # Initialization Function
    def __init__(self):

        # Parameters from YAML File
        self.drone_base_footprint_frame = rospy.get_param('drone_base_footprint')
        self.drone_base_link_frame = rospy.get_param('drone_base_link')
        self.map_frame = rospy.get_param('map_frame')
        self.control_drone_srv_name = rospy.get_param('drone_control_srv')

        # TF Listener for Getting Drone Position
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf)
        # String with Num. of Command that will be sent to Drone Service
        self.drone_command = ''
        # Control Drone Service
        self.control_drone = rospy.ServiceProxy(self.control_drone_srv_name, ControlDrone)

    # Get the Yaw Between Map and Drone Function
    def get_drone_yaw(self):
        # TODO: Read TF and Compute Yaw
        try:
            tf_ = self.tf.lookup_transform(self.drone_base_link_frame, self.map_frame,
                                           rospy.Time(0), rospy.Duration(1.0))
            (_, _, yaw) = euler_from_quaternion([tf_.transform.rotation.x, tf_.transform.rotation.y,
                                                 tf_.transform.rotation.z, tf_.transform.rotation.w])
            return 180-np.rad2deg(yaw)
        except tf.LookupException or tf.ConnectivityException or tf.ExtrapolationException:
            return -1

    # Get the Position in Map of Drone Function
    def get_drone_pose(self):

        # TODO: Read TF and Compute Yaw
        try:
            tf_ = self.tf.lookup_transform(self.map_frame, self.drone_base_link_frame,
                                           rospy.Time(0), rospy.Duration(1.0))
            (r1, r2, r3, _) = quaternion_matrix([tf_.transform.rotation.x, tf_.transform.rotation.y,
                                                 tf_.transform.rotation.z, tf_.transform.rotation.w])
            pose = np.asmatrix([r1[0:3], r2[0:3], r3[0:3]]) * \
                   np.asmatrix([[tf_.transform.translation.x],[tf_.transform.translation.y],[tf_.transform.translation.z]])
            (x, y, z) = (pose.item(0), pose.item(1), pose.item(2))
        except tf.LookupException or tf.ConnectivityException or tf.ExtrapolationException:
            return -1

        return x, y, z

    # Send SIGNAL to Control Drone Service to Execute Command
    def sent_control_command(self, command):

        rospy.wait_for_service(self.control_drone_srv_name)
        try:
            drone_execute_result = self.control_drone(command)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
        return drone_execute_result.status


# Faster Loops in Native C using Numba
# Used in Cffi Class
@jit(nogil=True)
def cp_brush(brush,y,rows,cols):

    for i in range(rows):
        for j in range(cols):
            brush[i][j] = y[i][j]
    return brush


@jit(nogil=True)
def cp_thinning(skeleton,y,rows,cols):

    for i in range(rows):
        for j in range(cols):
            skeleton[i][j] = y[i][j]
    return skeleton


@jit(nogil=True)
def cp_prune(skeleton,y,rows,cols):

    for i in range(rows):
        for j in range(cols):
            skeleton[i][j] = y[i][j]
    return skeleton


# Used in OgmOperations Class
@jit(nopython=True)
def ogm_blur(ogm,min_x,max_x,min_y,max_y):

    local_ogm = np.copy(ogm)

    for i in range(min_x,max_x):
        for j in range(min_y,max_y):
            if ogm[i][j] > 49:
                c = 0
            for ii in range(-1, 2):
                for jj in range(-1, 2):
                    if ogm[i + ii][j + jj] <= 49:
                        c += 1
                if c >= 4:
                    local_ogm[i][j] = 0
    return local_ogm


# Used in ImageMatching Class
@jit(nopython=True, parallel=True)
def ogm_reconstruct(initial_ogm, added_ogm, height, width,low_w,high_w,low_h,high_h):

    for h in range(0, height):
        for w in range(0, width):
            if low_w < w < high_w and low_h < h < high_h:
                if initial_ogm[h][w] != 0 and initial_ogm[h][w] != 255:
                    initial_ogm[h][w] = added_ogm[h-low_h][w-low_w]
    return initial_ogm


# Attention :: h_side, w_side MUST be Absolute Numbers
@jit(nopython=True, parallel=True)
def ogm_kl_reconstruct(ogm, new_data, h_side, w_side, h_p, w_p, c_th, s_th, map_min_w, map_max_w , map_min_h, map_max_h):

    for h in range(-h_side, h_side):
        for w in range(-w_side, w_side):
            # Calculate Pixel Points in Map
            h_new = h_p + int(np.rint(h * c_th - w * s_th))
            w_new = w_p + int(np.rint(w * c_th + h * s_th))
            # Check to be inside Map Container and Replace it Only if OGM Data is Unknown in this Location
            if map_min_h < h_new < map_max_h and  map_min_w < w_new < map_max_w and ogm[h_new][w_new] == 51:
                # It's Free
                if new_data[h + h_side][w + w_side] == 255:
                    ogm[h_new][w_new] = 0
                # It's Occupied
                elif new_data[h + h_side][w + w_side] == 0:
                    ogm[h_new][w_new] = 100
    return ogm

