from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
          package='cone_tracking',
          executable='tracking',
          output='screen',
          parameters=[{
            "yolo_weights": "src/cone_tracking/Yolo/weights/best.pt"  
          }]
        ),
        launch_ros.actions.Node(
          package='cone_tracking',
          executable='lidar_bbox',
          output='screen'
        )
    ])