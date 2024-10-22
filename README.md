![](logo.jpeg)

# ROS2 & PX4 Precision Landing with ArUco Markers
Master the integration of ROS2, PX4, and OpenCV to achieve precision landing using ArUco marker detection. This tutorial delves into how to leverage ROS2's robust communication framework and PX4's flight control system to implement highly accurate landings for autonomous drones. You'll learn how to configure your environment, process camera feeds, and detect ArUco markers in real-time, enabling your drone to land precisely at designated targets. Whether you're new to drone development or an experienced engineer, this guide provides a step-by-step approach to achieving reliable precision landing with seamless integration into your ROS2 and PX4 projects.
#### ArUco Markers
Aruco markers are square fiducial markers used in computer vision for tasks like pose estimation, camera calibration, and augmented reality (AR). Each marker has a unique binary pattern inside a black border, allowing it to be easily detected and identified. They help in determining the position and orientation of cameras or objects in a scene, making them valuable in robotics, navigation, and AR applications.
https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html

![](arucotag.png)


### Video Walkthrough
[Watch the video on YouTube](https://youtu.be/3BJB3OAo3tw?si=rJiTScTOzJzZjN2t)


### Prerequisites
* Ubuntu 22.04
* ROS2 Humble
* PX4 Autopilot with an ArUco Marker and downward facing camera
* Micro XRCE-DDS Agent
* QGroundControl Daily Build
* OpenCV 4.10.0
* ROS_GZ bridge

You can find the required instructions collected below

https://docs.px4.io/main/en/ros2/user_guide.html
https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/daily_builds.html

You need the lates PX4-Autopilot, that will contain the required drone with the downward facing camera and the world that has the aruco marker in it
To get ros_gz bridge
```
sudo apt install ros-humble-ros-gzgarden
```
https://github.com/gazebosim/ros_gz


For the OpenCV part follow the instructions below

## Usage

### Setup the Workspace
Make sure you source ROS2 Humble in the terminal you are using.
```
source /opt/ros/humble/setup.bash
```
OR
Just add the line above to your bashrc, in that case it is going to be sourced every time you open a terminal.
```
nano ~/.bashrc
```

Navigate to the directory you would like to place the worskpace and then run the following
```
git clone https://github.com/ARK-Electronics/tracktor-beam.git
```
Then navigate into the workspace:
```
cd tracktor-beam
```
Install OpenCV from source
```
./install_opencv.sh 
```
Install the submoduls
```
git submodule update --init --recursive
```
Build the workspace
```
colcon build
```
After this runs, we do not need to build the whole workspace again, you can just build the individual packages you have modified

```
colcon build --packages-select precision_land
```
Source the workspace
```
source install/setup.bash 
```
### Run the example

#### Run the simulation environment
Launch PX4 sim
```
make px4_sitl gz_x500_mono_cam_down_aruco
```
Launch micro dds
```
MicroXRCEAgent udp4 -p 8888
```

Launch the ros_gz_bridge for getting the camera topic
```
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```

Launch the ros_gz_bridge for getting the camera info topic (this is how we get camera intrinsics)
```
ros2 run ros_gz_bridge parameter_bridge /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

Launch the ros2 nodes (aruco_tracker)
```
cd tracktor-beam/
source install/setup.bash 
ros2 run aruco_tracker aruco_tracker 
```
OR
Launch file with the bridges:
```
source install/setup.bash
ros2 launch aruco_tracker aruco_tracker.launch.py 
```

View the video (/image_proc is the annoted image)
```
ros2 run rqt_image_view rqt_image_view
```

Launch the ros2 nodes (precision_land)
```
cd tracktor-beam/
source install/setup.bash 
ros2 run precision_land precision_land
```
OR
Launch file with the params:
```
ros2 launch precision_land precision_land.launch.py
```
Once the nodes are running the Precision Land mode is going to show up in QGC

![](Precision.png)

## ARK Electronics
For more open-source drone-related material, follow us on LinkedIn and Twitter:

[LinkedIn](https://www.linkedin.com/company/ark-electronics-llc/)

[X](https://x.com/ark_electr0nics)

If you're interested in US-manufactured drone hardware, please visit our webpage:

[ARK Electronics](https://arkelectron.com/)

## Questions
Message Patrik Dominik Pordi on the ARK Electronics / Dronecode Foundation Discord for questions or email me at patrik@arkelectron.com

[ARK Electronics Discord](https://discord.gg/un4HYu8k)


## Additional resources
[LinuxCheatSheet](https://www.geeksforgeeks.org/linux-commands-cheat-sheet/)

[ROS2CheatSheet](https://www.theconstruct.ai/wp-content/uploads/2021/10/ROS2-Command-Cheat-Sheets-updated.pdf)

[CMakeBasics](https://nu-msr.github.io/navigation_site/lectures/cmake_basics.html)

### Hardware
#### Prerequisites
* Ubuntu 22.04
* ROS2 Humble
* Drone with downward facing camera
* Micro XRCE-DDS Agent
* QGroundControl
* OpenCV 4.10.0
* Camera node:
I use an usb camera, there is a ROS2 package already out there for it:
https://github.com/ros-drivers/usb_cam.git

You can either run the the nodes or turn them into a service, that starts at boot:
#### Normal run

##### Service

First, you need to move your service file to the /etc/systemd/system/ directory, where systemd can find it. Replace myservice.service with the actual name of your service file.

Ensure that the service file has the correct permissions. Typically, it should be readable by all users:
```
sudo chmod 644 /etc/systemd/system/myservice.service

```
After copying the service file, reload the systemd daemon to recognize the new service:

```
sudo systemctl daemon-reload

```
Start the service using systemctl:

```
sudo systemctl start myservice

```
If you want the service to start automatically on boot, enable it:

```
sudo systemctl enable myservice

```
Verify that the service is running correctly:
```
sudo systemctl status myservice

```
