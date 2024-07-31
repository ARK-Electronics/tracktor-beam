from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    # Include the Aruco tracker launch file
    aruco_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('aruco_tracker'),
            'launch',
            'aruco_tracker.launch.py'
        ]))
    )



    # C++ nodes
    precision_land_cpp = Node(
        package="precision_land",
        executable="precision_land",
        parameters=[
            {"search_allowed": False},
        ]
    )

    # Add all actions to the launch description
    ld.add_action(aruco_tracker_launch)
    ld.add_action(precision_land_cpp)

    return ld
