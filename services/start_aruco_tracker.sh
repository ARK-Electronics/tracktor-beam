#!/bin/bash
source /opt/ros/humble/setup.bash
cd /home/jetson/code/ros2_jetpack6_ws/
source install/setup.bash
ros2 run aruco_tracker aruco_tracker
