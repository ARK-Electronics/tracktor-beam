import launch
from launch_ros.actions import Node

def generate_launch_description():

    usbcam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': '/dev/video2',
            'camera_info_url': '/home/jake/code/ark/offboard_ws/usb_cam_calib.yml',
        }]
    )

    tracker_node = Node(
        package='tracktorbeam',
        executable='tracker',
        name='tracker'
    )

    return launch.LaunchDescription([
        usbcam_node,
        tracker_node
    ])
