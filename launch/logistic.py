from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node


def generate_launch_description():

    locate_cones = Node(
        package = "mapping", 
        executable = "locate_cones",
        parameters = [{
            "max_rematch_distance": 0.20, 
            "cleanup_comparison_radius": 0.5,
            "cleanup_confidence_scale" : 0.3,
        }],
    )

    wall_detection = Node(
        package = "mapping", 
        executable = "wall_detector",
        parameters = [{
            "mode": "Logistic",
            "max_offset_angle": 0.3,
            "max_middle_offset": 0.2,
            "max_wall_length": 1.25,
            # "wall_pairs": '[["GREEN", "RED"]]'
        }],
    )

    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('src/m-explore-ros2/explore/launch/explore.launch.py'),
    )
                        
    return LaunchDescription([
         locate_cones,
         wall_detection,
#         exploration
    ])
