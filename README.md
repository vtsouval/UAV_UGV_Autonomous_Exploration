# UAV_UGV_Autonomous_Exploration

My Diploma Thesis in Aristotle University of Thessaloniki (AUTH) from April 2017 to March 2018 in Robotics.
 
Research in efficient ways of collaboration for an Unmanned Aerial Vehicle (UAV) and an Unmanned Ground Vehicle (UGV) for an autonomous exploration of unknown space.

The ROS Packages and Simulations have been tested under Ubuntu 14.04.5 LTS (Trusty Tahr)

# Summary

Read the Abstract inside the Thesis_Report.pdf file for a quick explanation of the collaboration techniques explored in this Thesis. For the investigation of the efficiency of those techniques 3 spaces has been created in Gazebo with a variety of obstacles.

# Pre-Installation

(Α) ROS Indigo and Gazebo 2.2.6 must be installed in the Ubuntu. Instruction for installation of ROS and Gazebo are given in http://wiki.ros.org/indigo/Installation/Ubuntu

(B) The following ROS Packages are prerequired for running the current Project.

•	Ar Pose - http://wiki.ros.org/ar_pose

This package is needed for detecting the Ar Pose placed in the top of the UGV (Turtlebot2) in order to be located by the UAV (ArDrone 2.0).

•	ArDrone Autonomy - http://wiki.ros.org/ardrone_autonomy
•	Tum Simulator  - https://github.com/dougvk/tum_simulator

The above two packages are required for the basic usage of the UAV (ArDrone 2.0) in ROS.

•	CRSM Slam - http://wiki.ros.org/crsm_slam

The Critical Rays Scan Match SLAM is used for the production of the final map of the unknown space.

•	Occupancy Grid Mapping Planner - https://github.com/etsardou/intelligent_robot_systems_2016/tree/master/art_ogmpp

This package is needed for the creation of the path between the UGV and the selected target point of the explored map during the exploration.


# Installation

1.	Insert the project into your catkin src folder (by default this is ~/catkin_ws/src).

2.	Copy files found inside the turtlebot_ar_marker folder and Move them inside the turtlebot_description package (Replace any file if necessary) to add the AR Marker on top of the UAV (Turtlebot2).

3.	Run the catkin_make command from terminal inside the catkin workspace to compile the projects.

4.	Execute the demo example’s by running :

•	rosrun experiments both_exploration.launch
To run the exploration of the unknown space with the collaboration of both UAV (ArDrone 2.0)  and UGV (Turtlebot2).

•	rosrun experiments turtlebot_exploration.launch
To run the exploration of the unknown space only with the UGV (Turtlebot2).


# Experimentation

Feel free to experiment with the current package. To change any parameters of exploration or run it in your own map change the .yaml files inside the YAML_Files folder of the project which is located inside the experiments package.

