from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Run bridge nodes in separate screen sessions
        ExecuteProcess(
            cmd=['screen', '-dmS', 'image_bridge', 'bash', '-c', 'ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image'],
            name='image_bridge_process'
        ),
        ExecuteProcess(
            cmd=['screen', '-dmS', 'camera_info_bridge', 'bash', '-c', 'ros2 run ros_gz_bridge parameter_bridge /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            name='camera_info_bridge_process'
        ),
        # Continue running Aruco tracker node as a normal ROS node and display output on screen
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen'
        ),
    ])