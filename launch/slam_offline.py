from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
          parameters=['config/mapper_params_offline.yaml'],
          package='slam_toolbox',
          executable='sync_slam_toolbox_node',
          name='slam_toolbox',
          #arguments=['--ros-args', '--log-level', "debug"],
          output='screen'
        )
    ])
