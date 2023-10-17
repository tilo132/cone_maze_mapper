from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription

import ament_index_python
from os import path

package_name = "cat_nav2"
share_dir = ament_index_python.get_package_share_directory(package_name)

def generate_launch_description():
    # launches the localization, mapper and planner launch files
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(share_dir, "localization_launch.py"))
    )
    mapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(share_dir, "mapper_launch.py"))
    )
    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(share_dir, "planner_launch.py"))
    )
    return LaunchDescription([
        #localization_launch,
        mapper_launch,
        planner_launch
    ])
