import launch
from launch_ros.actions import Node

def generate_launch_description():

    usbcam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[
            {'video_device': '/dev/video2'}
        ]
    )

    tracker_node = Node(
        package='tracktorbeam',
        executable='tracktorbeam',
        name='tracktorbeam'
    )

    return launch.LaunchDescription([
        usbcam_node,
        tracker_node
    ])
