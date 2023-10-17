from launch import LaunchDescription
import launch_ros.actions

import ament_index_python
from os import path

package_name = "cat_nav2"
share_dir = ament_index_python.get_package_share_directory(package_name)

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
          parameters=[path.join(share_dir, "config", "mapper_default.yaml")],
          package='slam_toolbox',
          executable='sync_slam_toolbox_node',
          name='slam_toolbox',
          output='screen'
        )
    ])