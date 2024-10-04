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

View the video (/image_proc is the annoted image)
```
ros2 run rqt_image_view rqt_image_view
```