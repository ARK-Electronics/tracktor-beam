![](logo.jpeg)

# ROS2 & PX4 Custom Precision Land example


This project is a customized example of the usage of the Auterions PX4-ROS2 Interface Library

https://github.com/Auterion/px4-ros2-interface-lib

### Video Walkthrough


### Prerequisites
* Ubuntu 22.04
* ROS2 Humble
* PX4 Autopilot
* Micro XRCE-DDS Agent
* QGroundControl Daily Build
* OpenCV built from source

You can find the required instructions collected below

https://docs.px4.io/main/en/ros2/user_guide.html


https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/daily_builds.html

You can also have a look on our previous tutorial where the steps above are covered:

https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example

To makes sure you are using the correct version of OpenCV please build it from source. You can use the instructions below to do so.

#### Set up the Workspace
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
Install the submoduls
```
git submodule update --init --recursive
```
Install and build OpenCV from source
```
./install_opencv.sh 
```
Build the workspace
```
colcon build
```
After this runs, we do not need to build the whole workspace again, you can just build the individual packages you have modified
```
colcon build --packages-select aruco_tracker precision_land
```
Source the workspace
```
source install/setup.bash 
```


## Usage

### Setup the Workspace


### Run the example

#### Run the simulation environment
We need to incorporate a downward-facing camera on the drone and add an Aruco marker to the custom world.
To do that you can do the following
```
cd PX4-Autopilot/Tools/simulation/gz
git remote add jake https://github.com/dakejahl/PX4-gazebo-models
git checkout pr-arucotag
```
Now we can launch the world with the ArUco marker and the drone with a downward facing camera on it.
```
cd PX4-Autopilot/
PX4_GZ_WORLD=aruco make px4_sitl_default gz_x500_mono_cam_down
```

#### Run the Micro XRCE-DDS Agent for the communication stream
```
MicroXRCEAgent udp4 -p 8888
```

#### Run QGC Daily build
Navigate to the directory
```
./QGroundControl.AppImage
```
Take off with the drone using the GUI

#### Launch your custom mode
I created a launch file that you can use. It currently contains only one node, so it might seem limited, but you can expand on it. The file includes three basic patterns: circle, spiral, and figure-8. These are ROS2 parameters that you can set either directly in the launch file or via command line arguments. If no pattern is specified, the default is circle.

```
cd px4_ros2_examples_ws/
source install/setup.bash 
```
AND
```
ros2 run custom_mode custom_mode
```
OR
```
ros2 launch custom_mode custom_mode.launch.py
```
OR
```
ros2 launch custom_mode custom_mode.launch.py trajectory_type:=spiral
```
OR
```
ros2 run custom_mode custom_mode --ros-args -p trajectory_type:=figure_8

```
You can view the bridges in different terminals
```
screen -r image_bridge
screen -r camera_info_bridge
```

#### Start it from QGC
You can just start the custom node from the GUI or you can also map it to your remote control

#### Closing remarks
Once you are done do not forget to close all your terminals

## Video

## ARK Electronics
For more open-source drone-related material, follow us on LinkedIn and Twitter:

[LinkedIn](https://www.linkedin.com/company/ark-electronics-llc/)

[X](https://x.com/ark_electr0nics)

If you're interested in US-manufactured drone hardware, please visit our webpage:

[ARK Electronics](https://arkelectron.com/)

## Questions
Message Patrik Dominik Pordi on the Dronecode Foundation Discord for questions or email me at patrik@arkelectron.com

## Additional resources
[LinuxCheatSheet](https://www.geeksforgeeks.org/linux-commands-cheat-sheet/)
[ROS2CheatSheet](https://www.theconstruct.ai/wp-content/uploads/2021/10/ROS2-Command-Cheat-Sheets-updated.pdf)
[CMakeBasics](https://nu-msr.github.io/navigation_site/lectures/cmake_basics.html)