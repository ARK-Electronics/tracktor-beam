#!/bin/bash
source /opt/ros/humble/setup.bash
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH


# Run the ROS2 usb_cam node with specified parameters
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p frame_id:=camera -p image_width:=1920 -p image_height:=1080 -p camera_info_url:=package://usb_cam/config/camera_info.yaml
