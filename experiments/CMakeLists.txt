cmake_minimum_required(VERSION 2.8.3)
project(experiments)

find_package(catkin REQUIRED  
	geometry_msgs
	nav_msgs
	roscpp
	rospy
	std_msgs
	tf
	visualization_msgs
	message_generation
)

catkin_package()
include_directories()

