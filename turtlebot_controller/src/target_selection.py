#!/usr/bin/env python

import rospy
import random
import numpy as np

from time import time, sleep
from math import hypot
from utilities import DroneCommunication, RvizHandler, OgmOperations
from brushfires import Brushfires
from topology import Topology
from path_planning import PathPlanning
# Costume Services and Messages Added
from drone_controller.srv import EvaluateTargets, EvaluateTargetsRequest
from drone_controller.msg import ColorEvaluationArray


# Class for selecting the next best target
class TargetSelection:

    # Constructor
    def __init__(self, selection_method):

        self.initial_time = time()
        self.method = selection_method
        self.initialize_gains = False

        self.brush = Brushfires()
        self.topology = Topology()
        self.path_planning = PathPlanning()
        self.droneConnector = DroneCommunication()

        # Parameters from YAML File
        self.debug = True #rospy.get_param('debug')
        self.map_discovery_purpose = rospy.get_param('map_discovery_purpose')
        self.color_evaluation_flag = rospy.get_param('color_rating')
        self.drone_color_evaluation_topic = rospy.get_param('drone_pub_color_rating')
        self.evaluate_potential_targets_srv_name = rospy.get_param('rate_potential_targets_srv')

        # Explore Gains
        self.g_color = 0.0
        self.g_brush = 0.0
        self.g_corner = 0.0
        self.g_distance = 0.0
        self.set_gain()

        if self.color_evaluation_flag:

            # Color Evaluation Service
            self.color_evaluation_service = rospy.ServiceProxy(self.evaluate_potential_targets_srv_name, EvaluateTargets)
            # Subscribe to Color Evaluation Topic to Get Results from Color Evaluation
            self.drone_color_evaluation_sub = rospy.Subscriber(self.drone_color_evaluation_topic,
                                                               ColorEvaluationArray, self.color_evaluation_cb)
            # Parameters
            self.targets_color_evaluated = False    # Set True Once Color Evaluation of Targets Completed
            self.color_evaluation = []              # Holds the Color Evaluation of Targets
            self.corner_evaluation = []             # Holds the Number of Corners Near Each Target

    # Target Selection Function
    def selectTarget(self, init_ogm, coverage, robot_pose, origin, resolution, g_robot_pose, previous_limits=[], force_random=False):

        # Initialize Target
        target = [-1, -1]
        if self.running_time() > 15:
            print ('\x1b[37;1m' + str('15 Minutes Constraint Passed!!!') + '\x1b[0m')

        # Find only the useful boundaries of OGM
        start = time()
        ogm_limits = OgmOperations.findUsefulBoundaries(init_ogm, origin, resolution, print_result=True, step=20)
        if self.debug:
            print ('\x1b[34;1m' + str('Target Selection: OGM Boundaries ') + str(ogm_limits)
                   + str(' in ') + str(time() - start) + str(' seconds.') + '\x1b[0m')

        # Blur the OGM to erase discontinuities due to laser rays
        start = time()
        ogm = OgmOperations.blurUnoccupiedOgm(init_ogm, ogm_limits)
        if self.debug:
            print ('\x1b[34;1m' + str('Target Selection: OGM Blurred in ') +
                   str(time() - start) + str(' seconds.') + '\x1b[0m')

        # Calculate Brushfire field
        start = time()
        brush = self.brush.obstaclesBrushfireCffi(ogm, ogm_limits)
        if self.debug:
            print ('\x1b[34;1m' + str('Target Selection: Brush in ') +
                   str(time() - start) + str(' seconds.') + '\x1b[0m')

        # Calculate Robot Position
        [rx, ry] = [robot_pose['x_px'] - origin['x'] / resolution, robot_pose['y_px'] - origin['y'] / resolution]

        # Calculate Skeletonization
        start = time()
        skeleton = self.topology.skeletonizationCffi(ogm, origin, resolution, ogm_limits)
        if self.debug:
            print ('\x1b[34;1m' + str('Target Selection: Skeletonization in ') +
                   str(time() - start) + str(' seconds.') + '\x1b[0m')

        # Find Topological Graph
        start = time()
        potential_targets = self.topology.topologicalNodes(ogm, skeleton, coverage, brush,
                                                           final_num_of_nodes=25, erase_distance=100, step=15)
        if self.debug:
            print ('\x1b[34;1m' + str('Target Selection: Topological Graph in ') +
                   str(time() - start) + str(' seconds.') + '\x1b[0m')
            print ('\x1b[34;1m' + str("The Potential Targets to be Checked are ")
                   + str(len(potential_targets)) + '\x1b[0m')

        if len(potential_targets) == 0:
            print ('\x1b[32;1m' + str('\n------------------------------------------')
                   + str("Finished Space Exploration!!! ")
                   + str('------------------------------------------\n') + '\x1b[0m')
            sleep(10000)

        # Visualization of Topological Graph
        vis__potential_targets = []
        for n in potential_targets:
            vis__potential_targets.append([n[0]*resolution + origin['x'], n[1]*resolution + origin['y']])
        RvizHandler.printMarker(vis__potential_targets, 1, 0, "map", "art_topological_nodes", [0.3, 0.4, 0.7, 0.5],0.1)


        # Check if we have given values to Gains
        if not self.initialize_gains:
            self.set_gain()

        # Random Point Selection if Needed
        if self.method == 'random' or force_random:

                # Get Distance from Potential Targets
                distance = np.zeros((len(potential_targets), 1), dtype=np.float32)
                for idx, target in enumerate(potential_targets):
                    distance[idx] = hypot(rx - target[0], ry - target[1])
                distance *= 255.0 / distance.max()

                path = self.selectRandomTarget(ogm, coverage, brush, ogm_limits,
                                                 potential_targets, distance,
                                                 resolution, g_robot_pose)

                if path is not None:
                    return path
                else:
                    return []

        # Sent Potential Targets for Color Evaluation (if Flag is Enable)
        if self.color_evaluation_flag:
            start_color = time()
            while not self.sent_potential_targets_for_color_evaluation(potential_targets, resolution, origin):
                pass

        # Initialize Arrays for Target Selection
        id = np.array(range(0,len(potential_targets))).reshape(-1, 1)
        brushfire = np.zeros((len(potential_targets), 1), dtype=np.float32)
        distance = np.zeros((len(potential_targets), 1), dtype=np.float32)
        color = np.zeros((len(potential_targets), 1), dtype=np.float32)
        corners = np.zeros((len(potential_targets), 1), dtype=np.float32)
        score = np.zeros((len(potential_targets), 1), dtype=np.float32)

        # Calculate Distance and Brush Evaluation
        start= time()
        for idx, target in enumerate(potential_targets):
            distance[idx] = hypot(rx-target[0], ry-target[1])
            brushfire[idx] = brush[target[0], target[1]]

        if self.debug:
            print ('\x1b[35;1m' + str('Distance and Brush Evaluation Calculated in ') +
                   str(time() - start) + str(' seconds.') + '\x1b[0m')

        # Wait for Color Evaluation to be Completed
        if self.color_evaluation_flag:
            while not self.targets_color_evaluated:
                pass
            color = np.array(self.color_evaluation).reshape(-1,1)
            corners = np.array(self.corner_evaluation,dtype=np.float64).reshape(-1, 1)
            # Reset Flag for Next Run
            self.targets_color_evaluated = False
            if self.debug:
                print ('\x1b[35;1m' + str('Color Evaluation Calculated in ') +
                       str(time() - start_color) + str(' seconds.') + '\x1b[0m')

        # Normalize Evaluation Arrays to [0, 255]
        distance *= 255.0 / distance.max()
        brushfire *= 255.0 / brushfire.max()
        if self.color_evaluation_flag:
            # color max is 640*320 = 204800
            color *= 255.0 / color.max()
            color = 255.0 - color
            corners *= 255.0 / corners.max()

        # Calculate Score to Choose Best Target
        # Final Array = [ Id. | [X] | [Y] | Color | Brush | Dist. | Num. of Corners | Score ]
        #                  0     1     2     3       4       5            6             7
        # Max is: 255 + 255 -  0  -  0  = +510
        # Min is:  0  +  0  - 255 - 255 = -510
        evaluation = np.concatenate((id, potential_targets, color,
                                     brushfire, distance, corners, score), axis=1)
        for e in evaluation:
            # Choose Gains According to Type of Exploration (Default is Exploration)
            if self.map_discovery_purpose == 'coverage':
                e[7] = self.g_color*e[3] + self.g_brush*e[4] - self.g_distance*e[5] - self.g_corner*e[6]
            elif self.map_discovery_purpose == 'combined':
                # Gains Change over Time
                self.set_gain()
                e[7] = self.g_color*e[3] + self.g_brush*e[4] - self.g_distance*e[5] - self.g_corner*e[6]
            else:
                e[7] = self.g_color*e[3] + self.g_brush*e[4] - self.g_distance*e[5] - self.g_corner*e[6]

        # Normalize Score to [0, 255] and Sort According to Best Score (Increasingly)
        evaluation[:, 7] = self.rescale(evaluation[:, 7],
                                        (-self.g_distance*255.0-self.g_corner*255.0),
                                        (self.g_color*255.0 + self.g_brush*255.0),
                                        0.0, 255.0)
        evaluation = evaluation[evaluation[:, 7].argsort()]

        # Best Target is in the Bottom of Evaluation Table Now
        target = [evaluation[[len(potential_targets) - 1], [1]], evaluation[[len(potential_targets) - 1], [2]]]


        # Print The Score of the Target Selected
        if len(previous_limits) != 0:
            if not previous_limits['min_x'] < target[0] < previous_limits['max_x'] and not\
                                    previous_limits['min_y'] < target[1] < previous_limits['max_y']:

                print ('\x1b[38;1m' + str('Selected Target was inside Explored Area.') + '\x1b[0m')

        print ('\x1b[35;1m' + str('Selected Target was ')
               + str(int(evaluation.item((len(potential_targets)-1), 0)))
               + str(' with score of ')
               + str(evaluation.item((len(potential_targets)-1), 7)) + str('.') + '\x1b[0m')

        return self.path_planning.createPath(g_robot_pose, target, resolution)

    # Send SIGNAL to Drone to Color Evaluate Potential Targets Function
    def sent_potential_targets_for_color_evaluation(self, targets, resolution, origin):

        # Calculate Position of Targets in Map
        targets = np.asarray(targets, dtype=np.float64)
        for target in targets:
            target[0] = target[0] * resolution + origin['x']
            target[1] = target[1] * resolution + origin['y']
        # Create Color Evaluation Request Message
        color_evaluation_request_msg = EvaluateTargetsRequest()
        color_evaluation_request_msg.pos_x = np.array(targets[:, 0])
        color_evaluation_request_msg.pos_y = np.array(targets[:, 1])
        # Call Service
        rospy.wait_for_service(self.evaluate_potential_targets_srv_name)
        try:
            response = self.color_evaluation_service(color_evaluation_request_msg)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return response

    # Drone Color Evaluation Callback Function that Returns the Color and Corner Evaluation of Targets
    def color_evaluation_cb(self, evaluation):

        if not self.targets_color_evaluated:
            self.color_evaluation = evaluation.color_weight
            self.corner_evaluation = evaluation.corner_number
            self.targets_color_evaluated = True

    # Exploration Choose Gains Function
    def set_gain(self):

        # TODO: Limit Enhance and Color Rating and Turtlebot
        '''
        # Full Coverage
        if self.map_discovery_purpose == 'coverage':
            if not self.initialize_gains:
                self.g_distance = 4.0
                self.g_color = 2.0
                self.g_brush = 1.0
                self.g_corner = 1.0
                self.initialize_gains = True
        # Full Coverage Under Time Pressure
        elif self.map_discovery_purpose == 'combined':
            # Check Time Passed
            time_passed = self.running_time()
            self.initialize_gains = True
            # Behave More Like Full Coverage Before the Hour
            if time_passed < 7:
                self.g_distance = 4.0 - 6.0*(time_passed/7)
                self.g_color = 2.0 + 0.0*(time_passed/7)
                self.g_brush = 1.0 - 0.0*(time_passed/7)
                self.g_corner = 1.0 + 1.0*(time_passed/7)
            # Behave Like Full Exploration After an Hour
            else:
                self.g_distance = 2.0
                self.g_color = 2.0
                self.g_brush = 1.0
                self.g_corner = 2.0
        # Full Exploration
        else:
            if not self.initialize_gains:
                self.g_distance = -2.0
                self.g_color = 2.0
                self.g_brush = 1.0
                self.g_corner = 2.0
                self.initialize_gains = True
        '''

        # TODO: Limit Enhance and Turtlebot
        '''
        # Limit Enhance
        if self.map_discovery_purpose == 'coverage':
            if not self.initialize_gains:
                self.g_distance = 4.0
                self.g_color = 1.0
                self.g_brush = 1.0
                self.g_corner = 1.0
                self.initialize_gains = True
        # Full Coverage Under Time Pressure
        elif self.map_discovery_purpose == 'combined':
            # Check Time Passed
            time_passed = self.running_time()
            self.initialize_gains = True
            # Behave More Like Full Coverage Before the Hour
            if time_passed < 7:
                self.g_distance = 4.0 - 6.0 * (time_passed / 7)
                self.g_color = 1.0 + 3.0 * (time_passed / 7)
                self.g_brush = 1.0 - 0.0 * (time_passed / 7)
                self.g_corner = 1.0 + 2.0 * (time_passed / 7)
            # Behave Like Full Exploration After an Hour
            else:
                self.g_distance = 2.0
                self.g_color = 4.0
                self.g_brush = 1.0
                self.g_corner = 3.0
        # Full Exploration
        else:
            if not self.initialize_gains:
                self.g_distance = -2.0
                self.g_color = 4.0
                self.g_brush = 1.0
                self.g_corner = 3.0
                self.initialize_gains = True
            '''

        # TODO: Turtlebot Only Parameters
        # Full Coverage
        if self.map_discovery_purpose == 'coverage':
            if not self.initialize_gains:
                self.g_distance = 8.0
                self.g_color = 1.0
                self.g_brush = 1.0
                self.g_corner = 1.0
                self.initialize_gains  = True
        # Full Coverage Under Time Pressure
        elif self.map_discovery_purpose == 'combined':
            # Check Time Passed
            time_passed = self.running_time()
            self.initialize_gains = True
            # Behave More Like Full Coverage Before the Hour
            if time_passed < 7:
                self.g_distance = 8.0 - 6.0 * (time_passed / 7)
                self.g_color = 1.0 + 3.0 * (time_passed / 7)
                self.g_brush = 1.0 - 0.0 * (time_passed / 7)
                self.g_corner = 1.0 + 2.0 * (time_passed / 7)
            # Behave Like Full Exploration After an Hour
            else:
                self.g_distance = 2.0
                self.g_color = 4.0
                self.g_brush = 1.0
                self.g_corner = 3.0
        # Full Exploration
        else:
            if not self.initialize_gains:
                self.g_distance = 1.5  # 8.0
                self.g_color = 4.0
                self.g_brush = 1.0
                self.g_corner = 3.0
                self.initialize_gains = True

    # Function that Returns the running time of Simulation (in Minutes)
    def running_time(self):
        return (time() - self.initial_time)//60

    # Function Rescaling Input Array to [Minimum, Maximum]
    @staticmethod
    def rescale(input, min_in, max_in, min_out, max_out):

        scale_factor = (max_out- min_out) / (max_in - min_in)
        add_factor = min_out - scale_factor*min_in
        input *= scale_factor
        input += add_factor
        return input

    # Random Target Selection Function
    def selectRandomTarget(self, ogm, coverage, brush_ogm, ogm_limits,
                           potential_targets, distance, resolution, g_robot_pose):

        print ('\x1b[35;1m' + str('Random Target Selection.') + '\x1b[0m')
        # Combine Two Arrays
        evaluation = np.concatenate((potential_targets, distance), axis=1)
        # Get Distance from Potential Targets
        evaluation = evaluation[evaluation[:, 2].argsort()]

        start = time()
        # Check Path to Potential Target if Available
        while not len(evaluation) == 0:
            # Check Closest Potential Target
            target_checked = evaluation[0, :2]
            # Delete Target to be Checked from Array
            evaluation = np.delete(evaluation, [0], axis=0)
            path = self.path_planning.createPath(g_robot_pose, target_checked, resolution)
            if not len(path) == 0:
                print ('\x1b[35;1m' + str('Select Closest Target in ') + str(time() - start) + '\x1b[0m')
                return path

        print ('\x1b[35;1m' + str('Select Closest Target Failed. '
                                  'Will Try Random Sampling inside OGM Limits...') + '\x1b[0m')
        # Check For Random Points inside Maps Limits
        failed_counter = 0
        while failed_counter < 1000:

            x_rand = random.randint(ogm_limits['min_x'], ogm_limits['max_x'])
            y_rand = random.randint(ogm_limits['min_y'], ogm_limits['max_y'])

            if ogm[x_rand][y_rand] < 50 and coverage[x_rand][y_rand] < 50 and brush_ogm[x_rand][y_rand] > 3:
                path = self.path_planning.createPath(g_robot_pose, [x_rand, y_rand], resolution)
                if not len(path) == 0:
                    print ('\x1b[35;1m' + str('Select Random Target in ') + str(time() - start) + '\x1b[0m')
                    return path
                # Increase Counter to Now When to Stop
                failed_counter += 1

        return None
