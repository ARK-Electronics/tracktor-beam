import launch
from launch_ros.actions import Node

def generate_launch_description():

    tracker_node = Node(
        package='tracktorbeam',
        executable='objecttracker',
        name='objecttracker',
        output='screen'
    )

    playback_node = Node(
        package='tracktorbeam',
        executable='videoplayback',
        name='videoplayback',
        output='screen'
    )

    return launch.LaunchDescription([
        tracker_node,
        playback_node
    ])
