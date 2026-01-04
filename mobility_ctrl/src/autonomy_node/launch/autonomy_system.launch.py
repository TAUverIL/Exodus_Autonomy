#!/usr/bin/env python3
"""
Complete Autonomy System Launch File
Launches all autonomy components for 3-camera perception, mapping, and path planning
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 3-Camera YOLO Detector
        Node(
            package='autonomy_node',
            executable='detector_3cam',
            name='detector_3cam',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),

        # Grid Mapper (builds occupancy grid from detections)
        Node(
            package='autonomy_node',
            executable='grid_mapper',
            name='grid_mapper',
            output='screen',
            parameters=[{
                'grid_resolution': 0.1,      # 10cm per cell
                'grid_width': 20.0,          # 20m wide
                'grid_height': 20.0,         # 20m tall
                'obstacle_radius': 0.3,      # 30cm inflation
                'decay_rate': 0.95,          # Gradual decay
                'use_sim_time': False
            }]
        ),

        # Odometry Fusion (Kalman filter for ZED + wheel odometry)
        Node(
            package='autonomy_node',
            executable='odom_fusion',
            name='odom_fusion',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),

        # A* Path Planner
        Node(
            package='autonomy_node',
            executable='path_planner',
            name='path_planner',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ])
