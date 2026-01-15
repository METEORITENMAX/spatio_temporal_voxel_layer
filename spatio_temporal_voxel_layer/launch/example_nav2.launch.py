#!/usr/bin/env python3

# Copyright (c) 2024 Steve Macenski
# Licensed under the LGPL-2.1 License

"""
Example launch file showing how to integrate the Spatio-Temporal Voxel Layer
with Nav2. This is a minimal example - you'll need to customize it for your robot.

Prerequisites:
1. You have a working Nav2 setup
2. You have sensor data (e.g., depth camera or 3D lidar) publishing PointCloud2
3. You have configured the plugin parameters in your costmap params file

Usage:
  ros2 launch spatio_temporal_voxel_layer example_nav2.launch.py

Note: This is an example. In practice, you would typically include this plugin
in your existing Nav2 bringup launch file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the path to the example config files
    pkg_share = FindPackageShare('spatio_temporal_voxel_layer')
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'example',
            'standard_indoor_environment_config.yaml'
        ]),
        description='Full path to the ROS2 parameters file for costmap configuration'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Note: This is a minimal example showing how to launch a costmap with STVL.
    # In a real robot setup, you would:
    # 1. Launch your full Nav2 stack (e.g., via nav2_bringup)
    # 2. Include the STVL plugin in your costmap configuration
    # 3. Make sure your sensor topics are publishing correctly

    # Example costmap node (you would typically use nav2_lifecycle_manager instead)
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('voxel_grid', 'voxel_grid'),
        ]
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        costmap_node,
    ])
