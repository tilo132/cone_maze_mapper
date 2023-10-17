import os
import math
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    camera_frame_id = "base_camera"
    param_url = LaunchConfiguration('param_url', default='config/realsense.yaml')
    rs = ComposableNode(
        name = "camera",
        package = "realsense2_camera",
        plugin = "realsense2_camera::RealSenseNodeFactory",
        parameters = [param_url],
        remappings=[
            ('/imu', '/rs/imu'), 
            ('color/image_raw', 'image_raw'), 
            ('color/image_raw/compressed', 'image_raw/compressed'),
            ('color/camera_info', 'camera_info'),
            ('color/metadata', 'camera_metadata'),
            ('extrinsics/depth_to_color', 'rs/extrinsics/depth_to_color')
        ]
    )

    container_name = 'camera_container'
    container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            rs
        ],
    )

    launch_arg = DeclareLaunchArgument(
        'fps',
        default_value='5',
        description="Fps for the camera node"
    )
    
    return LaunchDescription([
        launch_arg,
        container,
    ])
