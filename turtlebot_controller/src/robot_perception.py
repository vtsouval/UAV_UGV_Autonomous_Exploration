 #!/usr/bin/env python

import rospy
import numpy
import time
import tf
#from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException, TimeoutException
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from numba import jit

# Class implementing the robot perception: Reading the map, the coverage map
# and the robot pose
class RobotPerception:

    # Constructor
    def __init__(self):
        
        # Flags for debugging and synchronization
        self.print_robot_pose = False
        self.have_map = False
        self.map_token = False
        self.map_compute = False

        # Parameters from YAML File
        self.debug = rospy.get_param('debug')
        self.monitor_progress = rospy.get_param('save_progress')
        self.map_frame = rospy.get_param('map_frame')
        self.turtlebot_base_footprint_frame = rospy.get_param('turtlebot_base_footprint_frame')
        self.coverage_pub_topic = rospy.get_param('turtlebot_coverage_topic')
        self.ogm_topic = rospy.get_param('ogm_topic')
        self.robot_trajectory_topic = rospy.get_param('turtlebot_robot_trajectory_topic')
        self.cell_size = rospy.get_param('cell_size')
        self.near_field = rospy.get_param('near_field')
        self.resolution = rospy.get_param('map_resolution')

        # Holds the occupancy grid map
        self.ogm = 0
        self.ros_ogm = 0
        self.ogm_copy = 0

        # Holds the ogm info for copying reasons -- do not change
        self.ogm_info = 0
        self.prev_ogm_info = 0

        # Holds the robot's total path
        self.robot_trajectory = []
        self.previous_trajectory_length = 0

        # Holds the coverage information. This has the same size as the ogm
        # If a cell has the value of 0 it is uncovered
        # In the opposite case the cell's value will be 100
        self.coverage = []

        # Origin is the translation between the (0,0) of the robot pose and the
        # (0,0) of the map
        self.origin = {}
        self.origin['x'] = 0
        self.origin['y'] = 0

        # Initialization of robot pose
        # x,y are in meters
        # x_px, y_px are in pixels
        self.robot_pose = {}
        self.robot_pose['x'] = 0
        self.robot_pose['y'] = 0
        self.robot_pose['th'] = 0
        self.robot_pose['x_px'] = 0
        self.robot_pose['y_px'] = 0

        self.coverage_ogm = OccupancyGrid()
        self.coverage_ogm.header.frame_id = self.map_frame

        # Use tf to read the robot pose
        #self.tf = Buffer()
        #self.listener = TransformListener(self.tf)
        self.listener = tf.TransformListener()

        # Read robot pose with a timer
        rospy.Timer(rospy.Duration(0.11), self.readRobotPose)

        # ROS Subscriber to the occupancy grid map
        rospy.Subscriber(self.ogm_topic, OccupancyGrid, self.readMap)

        # Publisher of the robot trajectory
        self.robot_trajectory_publisher = rospy.Publisher(self.robot_trajectory_topic, Path, queue_size=10)

        # Publisher of the coverage field
        self.coverage_publisher = rospy.Publisher(self.coverage_pub_topic, OccupancyGrid, queue_size=10)

        # Read Cell size
        self.cell_matrix = numpy.zeros((1,1))
        self.current_cell = []

        # Parameters Used in Monitoring Progress
        self.map_size = 0
        self.unexplored_map = 0
        self.explored_map = 0
        self.percentage_explored = 0.0
        self.uncovered_map = 0
        self.covered_map = 0
        self.percentage_covered = 0.0
        self.time_passed = 0.0
        self.last_progress_check_time = 0.0
        self.num_of_explored_targets = 0.0

        if self.monitor_progress:
            # Create Files that Stores Data
            self.progress_file_name = rospy.get_param('progress_file_name')
            # Initialize .txt Files
            p_file = open(self.progress_file_name, "w")
            p_file.close()
            self.monitor_timer = rospy.Timer(rospy.Duration(60.0), self.monitoring_progress)

        print str('\n-----------------------------------------')
        print str('Robot Perception Initialized.')
        print str('Monitor Progress: ') + str(rospy.get_param('save_progress'))
        print str('Limits Exploration: ') + str(rospy.get_param('limit_exploration'))
        print str('Targets Color Evaluation: ') + str(rospy.get_param('color_rating'))
        print str('Save Image of Map: ') + str(rospy.get_param('turtlebot_save_progress_images'))
        print str('-----------------------------------------\n')

    # Getter for OGM. Must use flags since its update is asynchronous
    def getMap(self):

        print "Robot perception: Map requested"
        # The map is being processed ... waiting
        while self.map_compute:
            pass
        # Locking the map
        self.map_token = True
        # Copying it
        cp = cp_map(self.ogm)
        # Unlocking it
        self.map_token = False
        # Return the copy
        return cp

    def getRosMap(self):
        return self.ros_ogm

    # Getter for Coverage
    def getCoverage(self):
        return cp_coverage(self.coverage)

    # Reading the robot pose
    def readRobotPose(self, event):

        try:
            # Reads the robot pose from tf
            '''
            tf = self.tf.lookup_transform(self.map_frame, self.turtlebot_base_footprint_frame,
                                                               rospy.Time(0), rospy.Duration(1.0))
            translation = [tf.transform.translation.x, tf.transform.translation.y]
            rotation = [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]
            '''
            (translation, rotation) = self.listener.lookupTransform(self.map_frame, self.turtlebot_base_footprint_frame,
                                                                    rospy.Time(0))

        # Catch the exception if something is wrong
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.TimeoutException):
            print 'Error in tf'
            return

        # Updating the robot pose
        self.robot_pose['x'] = translation[0]
        self.robot_pose['y'] = translation[1]
        self.robot_pose['x_px'] = int(self.robot_pose['x'] / self.resolution)
        self.robot_pose['y_px'] = int(self.robot_pose['y'] / self.resolution)

        # Getting the Euler angles
        angles = tf.transformations.euler_from_quaternion(rotation)
        self.robot_pose['th'] = angles[2]

        # For debugging purposes
        if self.print_robot_pose:
            print self.robot_pose

        if [self.robot_pose['x'], self.robot_pose['y']] not in self.robot_trajectory:
            self.robot_trajectory.append([self.robot_pose['x'], self.robot_pose['y']])

        t_path = Path()
        t_path.header.frame_id = self.map_frame
        for p in self.robot_trajectory:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = p[0]
            ps.pose.position.y = p[1]
            t_path.poses.append(ps)
        self.robot_trajectory_publisher.publish(t_path)

    # Getting the occupancy grid map
    def readMap(self, data):

        # OGM is a 2D array of size width x height
        # The values are from 0 to 100
        # 0 is an unoccupied pixel
        # 100 is an occupied pixel
        # 50 or -1 is the unknown

        # Locking the map
        self.map_compute = True

        self.ros_ogm = data
        # Reading the map pixels
        self.ogm_info = data.info

        if not self.have_map or self.ogm_info.width != self.prev_ogm_info.width or\
                        self.ogm_info.height != self.prev_ogm_info.height:

            # Update Map Size for Monitoring Progress
            if self.monitor_progress:

                self.map_size = (rospy.get_param('x_max_size') - rospy.get_param('x_min_size'))*\
                                     (rospy.get_param('y_max_size')-rospy.get_param('y_min_size')) + 1500

                self.map_container = self.ogm_info.width * self.ogm_info.height - self.map_size

            print '\n\n----------------------------------------------------------'
            print 'Map & Coverage Expansion!'

            # Update coverage container as well
            self.ogm = numpy.zeros((data.info.width, data.info.height), dtype=numpy.int)
            self.prev_ogm_info = self.ogm_info
            coverage_copy = numpy.zeros([self.ogm_info.width, self.ogm_info.height])
            self.coverage_ogm.info = self.ogm_info
            self.coverage_ogm.data = numpy.zeros(self.ogm_info.width*self.ogm_info.height)

            # Create Copy of Coverage
            if self.have_map:
                start = time.time()
                coverage_copy = coverage_cp(coverage_copy, self.coverage,
                                            self.coverage.shape[0], self.coverage.shape[1])

                print 'Copying coverage field ' + str(self.coverage.shape) + str(' in ')\
                      + str(time.time() - start) + str(' seconds.')

            start = time.time()

            # Coverage Gets the New Size
            self.coverage = numpy.zeros([self.ogm_info.width, self.ogm_info.height])
            self.coverage = coverage_cp(self.coverage, coverage_copy,
                                        self.coverage.shape[0], self.coverage.shape[1])

            print "New coverage shape " + str(self.coverage.shape) \
                  + str(' and copied in ') + str(time.time() - start) + str(' seconds')

            # Copy Coverage Data
            start = time.time()
            self.coverage_ogm.data = coverage_ogm_cp(self.coverage_ogm.data, self.coverage,
                                                     self.coverage.shape[0], self.coverage.shape[1],
                                                     self.ogm_info.width)

            print str('Copied New Coverage Data in ') + str(time.time() - start) + str(' seconds.')
            print '----------------------------------------------------------\n\n'

        # Update OGM with New Data
        self.ogm = cp_ogm_data(self.ogm, numpy.array(data.data),
                               data.info.width, data.info.height)

        # Get the map's resolution - each pixel's side in meters
        self.resolution = numpy.around(data.info.resolution, decimals=3)

        # Get the map's origin
        self.origin['x'] = data.info.origin.position.x
        self.origin['y'] = data.info.origin.position.y

        # Keep a copy of OGM
        self.ogm_copy = numpy.copy(self.ogm)

        # Update Coverage
        xx = int(self.robot_pose['x_px'] + abs(self.origin['x']/self.resolution))
        yy = int(self.robot_pose['y_px'] + abs(self.origin['y']/self.resolution))

        for i in range(-self.near_field, self.near_field):
            for j in range(-self.near_field, self.near_field):
                if self.ogm[xx + i, yy + j] > 19 or self.ogm[xx + i, yy + j] == -1:
                    continue
                self.coverage[xx + i, yy + j] = 100
                index = int((xx + i) + self.ogm_info.width*(yy + j))
                self.coverage_ogm.data[index] = 100

        # Update the Progress Monitoring Parameters
        if self.monitor_progress:
            start = time.time()
            self.unexplored_map = len(self.ogm[numpy.where(self.ogm == 51)]) - self.map_container
            self.covered_map = len(self.coverage_ogm.data[numpy.where( self.coverage_ogm.data == 100)])
            if self.debug:
                print str('Progress Calculation Time ') + str(time.time() - start)

        # Publish Coverage
        self.coverage_publisher.publish(self.coverage_ogm)

        # Unlock the map
        self.map_compute = False

        # If it is copied wait ...
        while self.map_token:
            pass

        # This is for the Navigation and Monitoring Progress
        if not self.have_map:
            # Initialize ONCE when Map Received
            self.last_progress_check_time = time.time()
            self.have_map = True
            print str('Robot Perception: Map initialized')

    # Transforms relative coordinates to Global Function
    def getGlobalCoordinates(self, p, with_resolution = True):

        # If we want coordinates in pixels
        if with_resolution:
            return [p[0] - int(self.origin['x'] / self.resolution),
                    p[1] - int(self.origin['y'] / self.resolution)]

        # If we want the coordinates in meters
        else:
            return [p[0] - self.origin['x'], p[1] - self.origin['y']]

    # Monitoring Progress Function
    def monitoring_progress(self, event):

        # Start Monitoring After Map is Received
        if self.have_map:
            # If map is calculated, Wait...
            while self.map_compute:
                pass
            # Update the Progress Parameters
            self.explored_map = self.map_size - self.unexplored_map
            self.uncovered_map = self.map_size - self.covered_map
            self.percentage_explored = float(self.explored_map) / float(self.map_size)
            self.percentage_covered = float(self.covered_map)/float(self.map_size)
            self.time_passed += (time.time() - self.last_progress_check_time) / 60.0  # Save Time in Minutes
            # Write Result in Progress File
            store_progress_file = open(self.progress_file_name, "a")
            store_progress_file.write(str(self.time_passed) + str('\t') +
                                      str(self.unexplored_map) + str('\t') +
                                      str(self.explored_map) + str('\t') +
                                      str(self.percentage_explored) + str('\t') +
                                      str(self.uncovered_map) + str('\t') +
                                      str(self.covered_map) + str('\t') +
                                      str(self.percentage_covered) + str('\t') +
                                      str(self.map_size) + str('\t') +
                                      str(self.num_of_explored_targets) + str('\n'))
            store_progress_file.close()
            # Save Current Time for Next Timer Call
            self.last_progress_check_time = time.time()

            print ('\x1b[36;1m' + str('Coverage at ') + str(self.percentage_covered*100) + str(' and ') +
                   str('Exploration at ') + str(self.percentage_explored*100) + str(' in ') + str(self.time_passed) + str(' seconds.') +
                   str(self.num_of_explored_targets) + '\x1b[0m')

            if self.debug:
                print str('\n------------------------------------------------')
                print str('Map Size in Pixels: ') + str(self.map_size)
                print str('Map Container in Pixels: ') + str(self.map_container)
                print str('Number of Explored Space: ') + str(self.explored_map)
                print str('Number of Covered Space: ') + str(self.covered_map)
                print str('Number of Unexplored Space: ') + str(self.unexplored_map)
                print str('Number of Uncovered Space: ') + str(self.uncovered_map)
                print str('Exploration Complete Percentage: ') + str(self.percentage_explored)
                print str('Coverage Complete Percentage: ') + str(self.percentage_covered)
                print str('Time Passed (secs): ') + str(self.time_passed*60)
                print str('------------------------------------------------\n')


# Faster Loops in Native C using Numba
@jit('f8[:,:](f8[:,:],f8[:,:],i4,i4)', nopython=True, parallel=True)
def coverage_cp(coverage_out, coverage_in, width, height):

    for i in range(0, width):
        for j in range(0, height):
            coverage_out[i][j] = coverage_in[i][j]

    return coverage_out


@jit('f8[:](f8[:],f8[:,:],i4,i4,i4)', nopython=True, parallel=True)
def coverage_ogm_cp(coverage_ogm_data, coverage, width, height, ogm_width):

    for i in range(0, width):
        for j in range(0, height):
            index = int(i + width*j)
            coverage_ogm_data[index] = coverage[i][j]

    return coverage_ogm_data


@jit(nopython=True)
def cp_map(ogm):
    return numpy.copy(ogm)


@jit(nopython=True)
def cp_coverage(coverage):
    return numpy.copy(coverage)


@jit(nopython=True, parallel=True)
def cp_ogm_data(ogm,data,width,height):

    for i in range(0, width):
        for j in range(0, height):
            ogm[i][j] = data[i + width*j]

    return ogm