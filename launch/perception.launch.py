from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


# Launch Zed Camera

# Launch Lidar

def generate_launch_description():

    velodyne_driver_dir = get

    return LaunchDescription([
        # PointCloud to LaserScan (for costmaps)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{'config/pointcloud_to_laserscan.yaml'}],
            remappings=[
                ('cloud_in', '/zed/point_cloud'),
                ('scan', '/scan')
            ]
        ),
        
        # Custom sensor fusion node
        Node(
            package='zed_nav',
            executable='sensor_fusion.py',
            name='sensor_fusion'
        )
    ])