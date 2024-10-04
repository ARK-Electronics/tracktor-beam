#!/bin/bash
source /opt/ros/humble/setup.bash
cd /home/oem/ARK/tracktor-beam
source install/setup.bash
ros2 run aruco_tracker aruco_tracker
