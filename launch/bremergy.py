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
        parameters = [{"max_rematch_distance": 0.075}],
        output = "screen",
    )
    wall_detection = Node(
        package = "mapping", 
        executable = "wall_detector",
        parameters = [{"mode": "Bremergy"}],
        output = "screen",
        arguments=['--ros-args', '--log-level', "debug"],
    )
    explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource("src/m-explore-ros2/explore/launch/explore.launch.py")
    )
    
    return LaunchDescription([
         locate_cones,
         wall_detection,
         explore
    ])
