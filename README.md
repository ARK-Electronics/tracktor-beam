![](logo.jpeg)

# Installation and Dependencies

Clone the Repo and install the OpenCV dependencies
```
git clone --recursive  https://github.com/ARK-Electronics/tracktor-beam
cd tracktor-beam
./install_opencv.sh
```
Install `ros_gz_bridge`
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```
```
sudo apt update
sudo apt install ros-humble-ros-gz-bridge
```
Install  `rqt-image-view`
```
sudo apt install ros-$ROS_DISTRO-rqt-image-view
```
Build the ROS2 Packages
```
colcon build --packages-up-to precision_land
```

# Usage
Launch PX4 sim
```
make px4_sitl_default gz_x500_mono_cam_down_aruco
```
OR for multiple vehicle
```
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=x500_mono_cam_down ./build/px4_sitl_default/bin/px4 -i 1

```
AND
```
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=lawnmower_aruco ./build/px4_sitl_default/bin/px4 -i 2

```

Launch micro dds
```
MicroXRCEAgent udp4 -p 8888
```

Launch the ros_gz_bridge for briding the camera topic
```
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```

Launch the ros_gz_bridge for briding the camera info topic (this is how we get camera intrinsics)
```
ros2 run ros_gz_bridge parameter_bridge /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

Launch the ros2 nodes (aruco_tracker and precision_land)
```
ros2 launch precision_land precision_land.launch.py
```

View the video (/image_proc is the annoted image)
```
ros2 run rqt_image_view rqt_image_view
```
