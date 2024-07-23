![](logo.jpeg)
## Prerequisites

```
sudo apt-get install -y \
	ros-humble-usb-cam \
	ros-humble-image-view

```

```
ros2 launch tracktorbeam usbcam.launch.py
```
View the image
```
ros2 run image_view image_view --ros-args -r image:=/image_proc
```

### TODO:
Select USB cam using PID/VID
- udev rule?
- custom detection logic?

eg
```
v4l2-ctl --list-devices
Integrated_Webcam_HD: Integrate (usb-0000:00:14.0-11):
	/dev/video0
	/dev/video1
	/dev/media0

USB 2.0 Camera: USB Camera (usb-0000:00:14.0-3.2):
	/dev/video2
	/dev/video3
	/dev/media1

```

## Camera calibration
- fisheye calibration
- https://docs.ros.org/en/rolling/p/camera_calibration/tutorial_mono.html
USB camera
https://www.amazon.com/gp/product/B0829HZ3Q7/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1
```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2
```
```
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.015 --ros-args -r image:=/image_raw
```

bridge camera from gz to ros2
```
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```

bridge spawn entity service from gz to ros2
current unsupported in ros_gz (pending PR)
https://github.com/gazebosim/ros_gz/pull/380
```
ros2 run ros_gz_bridge parameter_bridge /world/default/create@ros_gz_interfaces/srv/SpawnEntity

```
