from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Run bridge nodes as separate processes
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen',
            name='image_bridge_process'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            output='screen',
            name='camera_info_bridge_process'
        ),
        # Continue running Aruco tracker node as a normal ROS node
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen'
        ),
        # Uncomment and modify this to run other nodes as needed
        # Node(
        #     package='precision_land',
        #     executable='precision_land',
        #     name='precision_land',
        #     output='screen'
        # ),
    ])
