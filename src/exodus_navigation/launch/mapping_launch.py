import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get the share directory of the slam_toolbox package
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Define the path to the standard slam_toolbox launch file
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    
    # Define the path to the rviz configuration file
    rviz_config_file = os.path.join(slam_toolbox_dir, 'config', 'slam_toolbox_default.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Include the slam_toolbox launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Start RViz2 with the slam_toolbox configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Convert 3D PointCloud2 to 2D LaserScan for SLAM
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': 0.1,
                'max_height': 1.5,
                'angle_min': -1.5708,  # -90.0 degrees
                'angle_max': 1.5708,   # 90.0 degrees
                'angle_increment': 0.0087,  # 0.5 degrees
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'use_sim_time': use_sim_time
            }],
            remappings=[('cloud_in', '/fused_pointcloud'),
                        ('scan', '/scan')]
        ),
    ])
