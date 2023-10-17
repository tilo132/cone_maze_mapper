import os
import math
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():        
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource("launch/camera.py")
    )
    yolo_node = Node(
        package = "cone_tracking",
        executable = "tracking",
        parameters=[{'overlay_bbox': True}],
    )
    qr_code_node = Node(
        package = "qr_code_scann",
        executable = "qr_code",
        parameters = [{"qr_code_side_length": 0.09}]
    )
    return LaunchDescription([
        camera_launch,
        yolo_node,
        qr_code_node
    ])
