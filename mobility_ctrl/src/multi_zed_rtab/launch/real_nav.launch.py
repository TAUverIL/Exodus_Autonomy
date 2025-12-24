import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('multi_zed_rtab')
    params_file = os.path.join(package_dir, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),

        # 1. Lifecycle Manager
        # Manages the two costmap nodes defined below.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['/local_costmap/local_costmap', '/global_costmap/global_costmap']}
            ]
        ),

        # 2. Local Costmap
        # Node Name: /local_costmap/local_costmap
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            namespace='local_costmap',
            output='screen',
            parameters=[params_file]
        ),

        # 3. Global Costmap
        # Node Name: /global_costmap/global_costmap
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            namespace='global_costmap',
            output='screen',
            parameters=[params_file]
        )
    ])