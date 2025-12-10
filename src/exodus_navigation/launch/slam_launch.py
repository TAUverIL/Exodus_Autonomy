import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generates the launch description for running SLAM.
    """
    # Get the path to the slam_toolbox package
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # --- Declare Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- SLAM Toolbox Launch ---
    # This starts the SLAM node in asynchronous mode
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([start_slam_toolbox_cmd])