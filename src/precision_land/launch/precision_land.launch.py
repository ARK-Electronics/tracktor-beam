from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # ExecuteProcess(
        #     cmd=['screen', '-dmS', 'dds_agent', 'bash', '-c', 'MicroXRCEAgent udp4 -p 8888'],
        #     name='dds_agent_process'
        # ),
        Node(
            package='precision_land',
            executable='precision_land',
            name='precision_land',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('precision_land'), 'cfg', 'params.yaml'])
            ]
        ),
    ])
