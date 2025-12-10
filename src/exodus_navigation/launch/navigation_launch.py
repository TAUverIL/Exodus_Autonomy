import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the directory of our package
    exodus_navigation_dir = get_package_share_directory('exodus_navigation')
    
    # Get the directory of nav2_bringup
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Define the path to our params file
    params_file = os.path.join(exodus_navigation_dir, 'params', 'nav2_params.yaml')
    
    # Configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Include the standard Nav2 navigation launch file
        # This starts the Costmap 2D nodes (Local and Global), Planner, and Controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true'
            }.items(),
        ),
    ])