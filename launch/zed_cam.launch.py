from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    zed_wrapper_pkg = get_package_share_directory('zed_wrapper')
    zed_launch = os.path.join(zed_wrapper_pkg, 'launch', 'zed_camera.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            launch_arguments={
                'camera_model': 'zed2i'
            }.items(),
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='odom_relay',
            arguments=['/zed/zed_node/odom', '/odom'],
            output='screen'
        )
    ])
