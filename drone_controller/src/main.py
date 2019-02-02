#!/usr/bin/env python

import rospy
from rosnode import kill_nodes
from time import sleep
from drone_control import Control_Drone
from geometry_msgs.msg import Twist


# The main function of the program
if __name__ == '__main__':

    # Wait for simulator and SLAM to initialize
    print "Waiting 6 seconds for initialization\n"
    sleep(6)

    # Initialize the ROS node
    rospy.init_node('drone_controller')
    # Creates a Control Drone object
    drone_control = Control_Drone()
    empty = Twist()

    #################################################
    # Part 1 - Drone Takeoff
    if not drone_control.state:
        while not drone_control.drone_takeoff():
            pass
    #################################################

    #################################################
    # Part 2 - Find Tag to Localize the Drone in Map
    drone_control.drone_localize_in_map()
    #################################################

    #################################################
    # Part 3 - Control Drone (Infinite Loop)
    '''
    Flags Used: - Control_Drone() -> marker_follow_flag Flag
                - Control_Drone() -> fly_flag Flag
                - Color_Evaluation() -> color_evaluate_targets_flag Flag
                - Limits_Exploration() -> drone_limit_exploration_flag Flag
    '''

    '''
    while True:

        if drone_control.marker_follow_flag:
            print ('Here')
            drone_control.marker_tracker()

        elif drone_control.color_evaluation.color_evaluate_targets_flag:

            if drone_control.fly_flag:
                print ('Here 2')
                while not drone_control.fly_to(drone_control.fly_direction):
                    pass
                drone_control.fly_direction = ''
                drone_control.fly_flag = False

            elif drone_control.color_evaluation.color_evaluate_targets_flag:
                print ('Here 3')
                drone_control.color_evaluation.\
                    color_evaluation_result_pub(drone_control.color_evaluation.evaluate_targets())
    '''
    while True:

        if drone_control.marker_follow_flag:
            drone_control.marker_tracker()

        '''
        first_run = True
        x_last = y_last = z_last = yaw_last = 0

        while drone_control.marker_follow_flag:

            # Check if any Service has been Called
            if drone_control.color_evaluation.color_evaluate_targets_flag or \
                    drone_control.limits_exploration.drone_limit_exploration_flag:
                drone_control.marker_follow_flag = False
                break
            # Target ON - Follow it!
            if drone_control.marker_follower.check_target_reached():
                # Get Pose Error
                [x, y,_, yaw,z] = drone_control.marker_follower.calculate_error_from_tag()
                # Height of Flight is Constant in height_down Parameter
                z = -z + drone_control.height_down
                # Run PID Initializer if First Time Flag is True
                if first_run:
                    drone_control.marker_follower.initialize_pid()
                # Publish Speed
                if not first_run:
                    drone_control.drone_speed_pub(drone_control.marker_follower.pose_pid(y + 0.1, y_last + 0.1, x, x_last, z, z_last, yaw, yaw_last))
                else:
                    first_run = False
                # Store Last Pose for Next Loop Run
                x_last = x
                y_last = y
                z_last = z
                yaw_last = yaw
            # Target OFF - Recovery State!
            else:
                drone_control.drone_speed_pub(drone_control.marker_follower.drone_marker_retrieval())
        '''

        if drone_control.fly_flag and not drone_control.color_evaluation.color_evaluate_targets_flag \
                and not drone_control.limits_exploration.drone_limit_exploration_flag:

            drone_control.fly_to(drone_control.fly_direction)
            drone_control.fly_direction = ''
            drone_control.fly_flag = False

        if drone_control.color_evaluation.color_evaluate_targets_flag \
                and not drone_control.limits_exploration.drone_limit_exploration_flag:

            drone_control.color_evaluation. \
                color_evaluation_result_pub.publish(drone_control.color_evaluation.evaluate_targets())
            drone_control.color_evaluation.color_evaluate_targets_flag = False

    #################################################