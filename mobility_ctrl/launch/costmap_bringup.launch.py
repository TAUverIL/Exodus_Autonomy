#!/usr/bin/env python3
"""
Exodus Autonomy - Costmap Bringup Launch File
==============================================
Launches Nav2 local and global costmaps connected to RTAB-Map SLAM output.

This launch file:
1. Starts global_costmap (uses RTAB-Map grid_map for static layer)
2. Starts local_costmap (uses camera point clouds for dynamic obstacles)
3. Activates lifecycle nodes for proper Nav2 integration
4. Connects RTAB-Map output (/rtabmap/grid_map) to costmap input

Prerequisites:
- RTAB-Map SLAM must be running and publishing /rtabmap/grid_map
- ZED cameras must be publishing point clouds
- TF tree must be properly configured (map->odom->base_link)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('mobility_ctrl')

    # Path to costmap configuration file
    costmap_config_file = PathJoinSubstitution([
        FindPackageShare('mobility_ctrl'),
        'config',
        'costmap_params.yaml'
    ])

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # === GLOBAL COSTMAP NODE ===
    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[
            costmap_config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # CRITICAL: Remap /map to RTAB-Map's grid_map output
            ('/map', '/rtabmap/grid_map'),
            # Ensure costmap publishes to correct topic
            ('costmap', 'global_costmap/costmap'),
            ('costmap_updates', 'global_costmap/costmap_updates'),
        ]
    )

    # === LOCAL COSTMAP NODE ===
    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[
            costmap_config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Local costmap topic remappings
            ('costmap', 'local_costmap/costmap'),
            ('costmap_updates', 'local_costmap/costmap_updates'),
            ('voxel_grid', 'local_costmap/voxel_grid'),
        ]
    )

    # === LIFECYCLE MANAGER ===
    # Manages lifecycle transitions for costmap nodes
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='costmap_lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},  # Automatically activate nodes on startup
            {'node_names': [
                'global_costmap',
                'local_costmap'
            ]}
        ]
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,

        # Costmap nodes
        global_costmap_node,
        local_costmap_node,

        # Lifecycle manager
        lifecycle_manager_node,
    ])
