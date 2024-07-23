Launch PX4 sim
```
make px4_sitl_default gz_x500_mono_cam_down
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


TODO: make launch file work. I want to run the above two ros_gz_bridge commands as nodes or execprocess but they block
the node from running.
Launch the ros2 nodes (aruco_tracker and precision_land)
```
```


View the video (/image_proc is the annoted image)
```
ros2 run rqt_image_view rqt_image_view
```