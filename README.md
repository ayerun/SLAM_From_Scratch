# SLAM & Differential Drive Software from Scratch
* Author: Arun Kumar

# Description
* This repository is a remake of the Turtlebot3 Burger ROS software
* See <a href="https://ayerun.github.io/Portfolio/SLAM.html" target="_blank">this portfolio post</a> for a detailed explanation
* Features
    * Differential Drive Kinematics
    * Dead Reckoning Odometry
    * Extended Kalman Filter SLAM

# Package List
This repository consists of several ROS packages
- nurtlesim - contains simulated world in rviz for Turtlebot3
- nuslam - contains a library for extended kalman filters and provides and implementation of SLAM
- nuturtle_description - contains urdf files, basic debugging, testing, and visualization code for the Turtlebot3
- nuturtle_robot - provides an interface for hardware on the Turtlebot3
- rigid2d - provides a C++ library for 2D transformations
- nuturtle_msgs - message definition for turtlebot sensors and wheel commands (written by Matthew Elwin)<br/>