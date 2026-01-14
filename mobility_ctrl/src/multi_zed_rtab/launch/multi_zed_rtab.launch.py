from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml

def load_yaml_params(context, config_path):
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    zed_config = config.get('zed_config', {})
    launch_flags = config.get('launch_flags', {})
    rtabmap_params = config.get('rtabmap_params', {})

    from ament_index_python.packages import get_package_share_directory
    rtabmap_ros_dir = get_package_share_directory('rtabmap_ros')
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)  # FALSE for real hardware!

    zed_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.expanduser('~'),
                'Exodus2025',
                'mobility_ctrl',
                'src',
                'zed-ros2-examples',
                'tutorials',
                'zed_multi_camera',
                'launch',
                'zed_multi_camera.launch.py'
            )
        ),
        launch_arguments=zed_config.items()
    )

    rgbd_sync1 = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        namespace='camera1',
        name='rgbd_sync1',
        remappings=[
            ('rgb/image', '/zed_multi/camera1/rgb/color/rect/image'),
            ('depth/image', '/zed_multi/camera1/depth/depth_registered'),
            ('rgb/camera_info', '/zed_multi/camera1/rgb/color/rect/camera_info'),
        ],
        parameters=[{'approx_sync': True},{'queue_size': 30}, {'use_sim_time':use_sim_time}]
    )

    rgbd_sync2 = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        namespace='camera2',
        name='rgbd_sync2',
        remappings=[
            ('rgb/image', '/zed_multi/camera2/rgb/color/rect/image'),
            ('depth/image', '/zed_multi/camera2/depth/depth_registered'),
            ('rgb/camera_info', '/zed_multi/camera2/rgb/color/rect/camera_info'),
        ],
        parameters=[{'approx_sync': True},{'queue_size': 30}, {'use_sim_time':use_sim_time}]
    )

    rgbd_sync3 = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        namespace='camera3',
        name='rgbd_sync3',
        remappings=[
            ('rgb/image', '/zed_multi/camera3/rgb/color/rect/image'),
            ('depth/image', '/zed_multi/camera3/depth/depth_registered'),
            ('rgb/camera_info', '/zed_multi/camera3/rgb/color/rect/camera_info'),
        ],
        parameters=[{'approx_sync': True},{'queue_size': 30}, {'use_sim_time':use_sim_time}]
    )

    rgbdx_sync = Node(
        package='rtabmap_sync',
        executable='rgbdx_sync',
        name='rgbdx_sync',
        namespace='rtabmap',
        remappings=[
            ('rgbd_image0', '/camera1/rgbd_image'),
            ('rgbd_image1', '/camera2/rgbd_image'),
            ('rgbd_image2', '/camera3/rgbd_image'),
            ('rgbd_images', '/rgbd_images'),
        ],
        parameters=[
            {'rgbd_cameras': 3},
            {'approx_sync': True},
            {'queue_size': 30}
        ]
    )

    odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='rtabmap',
        output='screen',
        remappings=[
            ('rgb/image', '/zed_multi/camera1/rgb/color/rect/image'),
            ('depth/image', '/zed_multi/camera1/depth/depth_registered'),
            ('rgb/camera_info', '/zed_multi/camera1/rgb/color/rect/camera_info'),
            ('rgbd_image', '/camera1/rgbd_image'),
            ('rgbd_images', '/rgbd_images'),
            ('imu', '/zed_multi/camera1/imu/data'),
        ],
        parameters=[rtabmap_params, {'use_sim_time':use_sim_time}]
    )
    
    rtabmap_node = Node(
    package='rtabmap_slam',
    executable='rtabmap',
    name='rtabmap',
    namespace='rtabmap',
    output='screen',
    arguments=['--delete_db_on_start'],
    remappings=[
        ('map', '/rtabmap/map'),
        ('rgb/image', '/zed_multi/camera1/rgb/color/rect/image'),
        ('depth/image', '/zed_multi/camera1/depth/depth_registered'),
        ('rgb/camera_info', '/zed_multi/camera1/rgb/color/rect/camera_info'),
        ('rgbd_image', '/camera1/rgbd_image'),
        ('rgbd_images', '/rgbd_images'),
        ('left/image_rect', '/zed_multi/camera1/rgb/color/rect/image'),
        ('right/image_rect', '/zed_multi/camera2/rgb/color/rect/image'),
        ('left/camera_info', '/zed_multi/camera1/rgb/color/rect/camera_info'),
        ('right/camera_info', '/zed_multi/camera2/rgb/color/rect/camera_info'),
        ('scan', '/scan'),
        ('scan_cloud', '/scan_cloud'),
        ('user_data', '/user_data'),
        ('user_data_async', '/user_data_async'),
        ('gps/fix', '/gps/fix'),
        ('tag_detections', '/detections'),
        ('fiducial_transforms', '/fiducial_transforms'),
        ('odom', '/rtabmap/odom'),
        ('imu', '/zed_multi/camera1/imu/data'),
        ('goal_out', '/goal_pose'),
    ],
    parameters=[rtabmap_params,{'use_sim_time':use_sim_time}]
    )

    rtabmapviz_node = Node(
    condition=IfCondition(str(launch_flags.get('rtabmapviz', 'true')).lower()),
    package='rtabmap_viz',
    executable='rtabmap_viz',
    name='rtabmap_viz',
    output='screen',
    arguments=['-d', os.path.join(rtabmap_ros_dir, 'launch', 'config', 'rgbd_gui.ini')],
    remappings=[
        ('rgb/image', '/zed_multi/camera1/rgb/color/rect/image'),
        ('depth/image', '/zed_multi/camera1/depth/depth_registered'),
        ('rgb/camera_info', '/zed_multi/camera1/rgb/color/rect/camera_info'),
        ('rgbd_image', '/camera1/rgbd_image'),
        ('rgbd_images', '/rgbd_images'),
        ('left/image_rect', '/zed_multi/camera1/rgb/color/rect/image'),
        ('right/image_rect', '/zed_multi/camera2/rgb/color/rect/image'),
        ('left/camera_info', '/zed_multi/camera1/rgb/color/rect/camera_info'),
        ('right/camera_info', '/zed_multi/camera2/rgb/color/rect/camera_info'),
        ('scan', '/scan'),
        ('scan_cloud', '/scan_cloud'),
        ('odom', '/rtabmap/odom'),
    ],
    parameters=[rtabmap_params,{'use_sim_time':use_sim_time}]
)

    rviz_node = Node(
        condition=IfCondition(str(launch_flags.get('rviz', 'false')).lower()),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(rtabmap_ros_dir, 'launch', 'config', 'rgbd.rviz')],
    )

    # Static Transform Publishers for TF tree
    # All cameras are children of base_link
    # TF tree: map -> odom -> base_link -> camera_links
    # Camera 1 (front): mounted 1.5m above base_link, facing forward
    tf_base_to_camera1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera1',
        arguments=['0.3', '0', '1.5', '0', '0', '0', 'base_link', 'camera1_camera_link'],
    )

    # Camera 2 (rear) - attached to base_link, facing backward
    tf_base_to_camera2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera2',
        arguments=['-0.12', '0', '1.0', '0', '0', '3.14159', 'base_link', 'camera2_camera_link'],
    )

    # Camera 3 (manipulator/overhead) - attached to base_link, 30deg down tilt
    tf_base_to_camera3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_camera3',
        arguments=['0.2', '0', '1.2', '0', '0.524', '0', 'base_link', 'camera3_camera_link'],
    )

    return [
        zed_launcher,
        rgbd_sync1,
        rgbd_sync2,
        rgbd_sync3,
        rgbdx_sync,
        odometry_node,
        rtabmap_node,
        rtabmapviz_node,
        rviz_node,
        tf_base_to_camera1,
        tf_base_to_camera2,
        tf_base_to_camera3
    ]

def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory

    # Get config file from installed package location
    pkg_share = get_package_share_directory('multi_zed_rtab')
    default_config = os.path.join(pkg_share, 'config', 'multi_camera_config.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to YAML config file.'
    )

    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=lambda context: load_yaml_params(
            context,
            LaunchConfiguration('config_file').perform(context)
        ))
    ])
