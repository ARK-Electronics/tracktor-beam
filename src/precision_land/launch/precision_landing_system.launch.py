from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch argument for enabling visualization
    enable_viz_arg = DeclareLaunchArgument(
        'enable_gazebo_viz',
        default_value='true',
        description='Enable Gazebo marker visualization (simulation only)'
    )

    return LaunchDescription([
        enable_viz_arg,
        # Bridge camera image from Gazebo to ROS2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='image_bridge',
            arguments=[
                '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image'
            ],
            output='screen',
        ),
        # Bridge camera info from Gazebo to ROS2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            arguments=[
                '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            output='screen',
        ),
        # Bridge processed image from ROS2 to Gazebo for visualization
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='image_proc_bridge',
            arguments=[
                '/image_proc@sensor_msgs/msg/Image[gz.msgs.Image'
            ],
            output='screen',
        ),
        # Aruco tracker node
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('aruco_tracker'), 'cfg', 'params.yaml'])
            ]
        ),
        # Precision landing node
        Node(
            package='precision_land',
            executable='precision_land',
            name='precision_land',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('precision_land'), 'cfg', 'params.yaml'])
            ]
        ),
        # Gazebo visualization node (simulation only)
        Node(
            package='precision_land_viz',
            executable='tag_pose_visualizer',
            name='tag_pose_visualizer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_gazebo_viz'))
        ),
    ])
