from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import ament_index_python
from os import path

package_name = "cat_nav2"
share_dir = ament_index_python.get_package_share_directory(package_name)

def generate_launch_description():
    param_url = LaunchConfiguration(
    	'param_url',
    	default=path.join(share_dir, 'config/ekf.yaml')
    )
    localization = Node(
        package = "robot_localization",
        executable = "ekf_node",
        parameters = [param_url]
    )
    return LaunchDescription([
        localization
    ])
