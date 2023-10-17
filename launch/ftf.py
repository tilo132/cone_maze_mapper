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
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():

    turtlebot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource("launch/turtlebot.py")
    )
                
    lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource("launch/lidar.py")
    )

    camera_launch =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource("launch/camera.py")
    )

    not_just_base_nodes = LaunchConfiguration('full', default=True)

    lidar_bb = Node(
        package = "cone_tracking",
        executable = "lidar_bbox",
        name = "lidar_bb",
        parameters = [{
            "laserscan_height_threshold": 0.1
        }],
        condition=IfCondition(not_just_base_nodes)
    )

    relay_on = LaunchConfiguration('relay', default=True)
   
    relay = Node(
        package = "topic_tools",
        executable = "relay",
        name = "relay_image_bb",
        parameters = [{
            "input_topic": "/image_bb/compressed",
            "output_topic": "/relay/image_bb/compressed"
        }],
        condition=IfCondition(relay_on)
    )
    relay_image_raw = Node(
        package = "topic_tools",
        executable = "relay",
        name = "relay_image_raw",
        parameters = [{
            "input_topic": "/image_raw/compressed",
            "output_topic": "/relay/image_raw/compressed"
        }],
        condition=IfCondition(relay_on)
    )
    relay_camera_info = Node(
        package = "topic_tools",
        executable = "relay",
        name = "relay_camera_info",
        parameters = [{
            "input_topic": "/camera_info",
            "output_topic": "/relay/camera_info"
        }],
        condition=IfCondition(relay_on)
    )
    relay_goal = Node(
            package = "topic_tools",
            executable = "relay",
            name = "relay_rviz_goal",
            parameters = [{
                "input_topic": "/move_base_simple/goal",
                "output_topic": "/goal_pose"
            }],
            condition=IfCondition(relay_on)
    )
    relay_poi = Node(
                package = "topic_tools",
                executable = "relay",
                name = "relay_poi",
                parameters = [{
                    "input_topic": "/poi",
                    "output_topic": "/relay/poi"
                }],
                condition=IfCondition(relay_on)
    )
    

    odom = IncludeLaunchDescription(PythonLaunchDescriptionSource("launch/robot_localisation.py"))
    
    return LaunchDescription([
        lidar_launch,
        lidar_bb,
        odom,
        turtlebot_bringup,
        relay,
#        relay_image_raw,
        relay_goal,
        relay_camera_info,
        relay_poi
    ])
