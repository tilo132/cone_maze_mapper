# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch the velodyne driver, pointcloud, and laserscan nodes with default configuration."""

import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions
import math

def generate_launch_description():

    print("---------------- Lidar Launch File ----------------")
    qos_overrides = {
        "qos_overrides./velodyne_points.publisher.depth": 1,
        "qos_overrides./velodyne_points.publisher.reliability": "best_effort"
    }
    velodyne_frame = "velodyne_link"
    driver_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP16-velodyne_driver_node-params.yaml')
    with open(driver_params_file, 'r') as f:
        driver_params = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']

    velodyne_driver_node = launch_ros.actions.Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='both',
        parameters=[
            driver_params,
            {
                "rpm": 300.0,
                "frame_id": velodyne_frame
            }
        ]
    )

    convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(convert_share_dir, 'config', 'VLP16-velodyne_convert_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        default_convert_params = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']

    convert_params = {
        **default_convert_params,
        "calibration": os.path.join(convert_share_dir, 'params', 'VLP16db.yaml'),
        "view_width": 180 / 360 * math.pi * 2,
        "view_direction": 0.0,
        "min_range": 0.1,
        "max_range": 5.0,
        "fixed_frame": velodyne_frame,
        "target_frame": velodyne_frame,
        **qos_overrides
    } 
    
    velodyne_convert_node = launch_ros.actions.Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        output='both',
        parameters=[convert_params]
    )

    laserscan_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_laserscan')
    laserscan_params_file = os.path.join(laserscan_share_dir,'config','default-velodyne_laserscan_node-params.yaml')
    with open(laserscan_params_file, 'r') as f:
        default_laser_params = yaml.safe_load(f)['velodyne_laserscan_node']['ros__parameters']
        
    laser_params = {
        **default_laser_params,
     "qos_overrides./scan.publisher.depth": 1,
      "qos_overrides./scan.publisher.reliability": "best_effort"
    }

    #lidar_filter = launch_ros.actions.Node(
    #    package="lidar_filter",
    #    executable="lidar_filter",
    #    parameters=[{
    #       "min_z" : 0.035
    #    }]
    #)


    return launch.LaunchDescription([
        velodyne_driver_node,
        velodyne_convert_node,
        # lidar_filter,

        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=velodyne_driver_node,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown()
                )],
            )),
        ])
