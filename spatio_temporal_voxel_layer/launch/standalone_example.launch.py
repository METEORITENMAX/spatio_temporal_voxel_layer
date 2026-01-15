#!/usr/bin/env python3

# Copyright (c) 2024 Steve Macenski
# Licensed under the LGPL-2.1 License

"""
Minimal standalone example launch file for testing the Spatio-Temporal Voxel Layer.

This demonstrates the plugin running in isolation with a simple costmap node.
Useful for testing and understanding how the plugin works.

Prerequisites:
- A depth camera or 3D lidar publishing on 'camera/depth/points' (or modify the topic)
- TF frames properly set up (map, base_link)

Usage:
  ros2 launch spatio_temporal_voxel_layer standalone_example.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Inline parameters for a minimal costmap with STVL
    costmap_params = {
        'use_sim_time': use_sim_time,
        'global_frame': 'map',
        'robot_base_frame': 'base_link',
        'update_frequency': 5.0,
        'publish_frequency': 2.0,
        'transform_tolerance': 0.5,
        'width': 10,
        'height': 10,
        'resolution': 0.05,
        'plugins': ['stvl_layer'],
        'stvl_layer': {
            'plugin': 'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer',
            'enabled': True,
            'voxel_decay': 15.0,
            'decay_model': 0,
            'voxel_size': 0.05,
            'track_unknown_space': True,
            'mark_threshold': 0,
            'update_footprint_enabled': True,
            'combination_method': 1,
            'origin_z': 0.0,
            'publish_voxel_map': True,
            'transform_tolerance': 0.2,
            'mapping_mode': False,
            'map_save_duration': 60.0,
            'observation_sources': ['pointcloud'],
            'pointcloud': {
                'data_type': 'PointCloud2',
                'topic': '/camera/depth/points',
                'marking': True,
                'clearing': True,
                'min_obstacle_height': 0.0,
                'max_obstacle_height': 2.0,
                'obstacle_range': 3.0,
                'min_z': 0.1,
                'max_z': 2.0,
                'vertical_fov_angle': 0.8745,
                'horizontal_fov_angle': 1.048,
                'decay_acceleration': 5.0,
                'model_type': 0,
            }
        }
    }

    # Costmap node
    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='costmap',
        output='screen',
        parameters=[costmap_params]
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        costmap_node,
    ])
