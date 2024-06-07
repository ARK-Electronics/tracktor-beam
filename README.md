![](small_logo.jpeg)
## Prerequisites

```
sudo apt-get install -y \
	ros-humble-usb-cam \
	ros-humble-image-view

```

```
ros2 launch tracktor-beam usbcam.launch.py
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
