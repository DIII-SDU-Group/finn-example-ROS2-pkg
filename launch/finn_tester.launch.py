import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    finn_interfacer = Node(
        package="finn-example",
        executable="finn_interfacer",
        name="finn_interfacer",
        namespace="finn_interfacer",
        remappings=[("/image", "/usb_cam/image_raw")]
    )

    config = os.path.join(
        get_package_share_directory('finn-example'),
        'config',
        'params.yaml'
    )

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        namespace="usb_cam",
        parameters=[config]
    )

    return LaunchDescription([
        finn_interfacer,
        camera_node
    ])
