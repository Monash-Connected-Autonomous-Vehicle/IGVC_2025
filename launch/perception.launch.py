from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch Zed Camera
    # Launch Lidar

    # Define all launch descriptions and nodes first
    velodyne_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('velodyne_pointcloud'),
                'launch',
                'velodyne_transform_node-VLP16-launch.py'
            )
        )
    )

    voxel_filter_node = Node(
        package='voxel_filter',
        executable='voxel_filter_node',
        name='voxel_filter_node',
        output='screen'
    )

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            )
        ),
        launch_arguments={'camera_model': 'zed2i'}.items()
    )

    return LaunchDescription([
        # Velodyne Driver Node
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            parameters=[{
                'device_ip': '192.168.10.201',  # UA default, override with launch argument
                'gps_time': False,
                'time_offset': 0.0,
                'enabled': True,
                'read_once': False,
                'read_fast': False,
                'repeat_delay': 0.0,
                'frame_id': 'velodyne',
                'model': 'VLP16',
                'rpm': 600.0,
                'port': 2368,  # UA default, override with launch argument
                'timestamp_first_packet': False,
                'cut_angle': -0.01  # Keeping your original additional parameter
            }],
            output='screen'
        ),

        # Velodyne Pointcloud Node
        velodyne_transform_launch,
        
        # Voxel filter Node
        voxel_filter_node,

        # Zed camera launch
        zed_wrapper_launch,

        # PointCloud to LaserScan (for costmaps)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',  # You might want to adjust this
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -3.1415,  # -pi
                'angle_max': 3.1415,    # pi
                'angle_increment': 0.0087,  # ~0.5 degree
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            remappings=[
                ('cloud_in', '/zed/point_cloud'),
                ('scan', '/scan')
            ],
            output='screen'
        ),

    ])