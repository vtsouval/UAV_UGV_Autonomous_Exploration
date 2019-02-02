#!/usr/bin/env python

from rospy import get_param, Subscriber, Service, is_shutdown, Time, Duration, init_node, Rate
from time import sleep
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import  TransformStamped
# Custom Messages and Services Added
from drone_controller.srv import JointTrees


# Class for TF Connection
class TF_Publisher:

    # Constructor
    def __init__(self):

        # Read Parameters From YAML File
        self.turtlebot_odom_topic = get_param('turtlebot_odom_topic')
        self.turtlebot_base_footprint_frame = get_param('turtlebot_base_footprint_frame')
        self.map_frame = get_param('map_frame')
        self.drone_odom_frame = get_param('drone_odom_frame')
        self.join_tree_srv_name = get_param('join_tree_srv')
        self.debug = get_param('debug')
        # TF Broadcaster
        self.tf_broadcast = TransformBroadcaster()
        # TF Listener
        self.tf_listen = Buffer()
        self.tf_listener = TransformListener(self.tf_listen)
        # Subscribers
        Subscriber(self.turtlebot_odom_topic, Odometry, self.turtlebot_odom_cb)
        # Services for Joining the Drone TF Tree to Main Tree (Localization)
        self.join_tree_service = Service(self.join_tree_srv_name, JointTrees, self.join_tree_srv_function)
        # Parameters
        self.drone_tf = TransformStamped()
        self.turtlebot_tf = TransformStamped()
        self.turtlebot_tf.header.frame_id = self.map_frame
        self.turtlebot_tf.child_frame_id = self.turtlebot_base_footprint_frame

        self.drone_tf.header.frame_id = self.map_frame
        self.drone_tf.child_frame_id = self.drone_odom_frame
        self.drone_tf.transform.rotation.w = 1.0
        self.turtlebot_tf.transform.rotation.w = 1.0

        # Flags
        self.join_trees = False # Set True When The Two TF Trees are Joint in One

    # Callback Function to Get Data from Turtlebot Odometry Topic
    def turtlebot_odom_cb(self, odometry):

        self.turtlebot_tf.transform.translation.x = odometry.pose.pose.position.x
        self.turtlebot_tf.transform.translation.y = odometry.pose.pose.position.y
        self.turtlebot_tf.transform.translation.z = odometry.pose.pose.position.z
        self.turtlebot_tf.transform.rotation.x = odometry.pose.pose.orientation.x
        self.turtlebot_tf.transform.rotation.y = odometry.pose.pose.orientation.y
        self.turtlebot_tf.transform.rotation.z = odometry.pose.pose.orientation.z
        self.turtlebot_tf.transform.rotation.w = odometry.pose.pose.orientation.w

    # Service Handle Function to Join Drone to Main Tree after Localization
    def join_tree_srv_function(self, command):

        if self.debug:
            print('\x1b[33;1m' + str('Joining the Two Separate Frame Tree!') + '\x1b[0m')
        # Set Flag to True
        self.join_trees = command.status
        # Check if we can calculate the TF Between Drones Bottom Camera and Ar Tag
        while not self.tf_listen.can_transform(self.drone_odom_frame, self.map_frame, Time(0), Duration(1.0)):
            pass
        # Return From Service Once Tree is Joint
        return True


if __name__ == '__main__':

    init_node('tf2_map_broadcaster')
    # Wait for Simulation to Start
    sleep(2.0)
    # Create a TF_connector Object
    tf_ = TF_Publisher()
    # Node Rate
    rate = Rate(100.0)
    # Stay in Loop and Broadcast TF's
    while not is_shutdown():

        tf_.turtlebot_tf.header.stamp = Time.now()
        tf_.tf_broadcast.sendTransform(tf_.turtlebot_tf)

        if tf_.join_trees:
            tf_.drone_tf.header.stamp = Time.now()
            tf_.tf_broadcast.sendTransform(tf_.drone_tf)

        rate.sleep()

