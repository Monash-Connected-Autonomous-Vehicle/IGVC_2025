from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Assuming your URDF file is in a package named "IGVC_2025"
    pkg_share = get_package_share_directory('IGVC_2025')
    urdf_path = os.path.join(pkg_share, 'urdf', 'zed_robot.urdf.xacro')
    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Robot State Publisher for URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'publish_frequency': 30.0
            }]
        ),
        
        # Odometry from ZED (using zed-ros2-wrapper)
        Node(
            package='zed_odom',
            executable='zed_odom',
            name='zed_odometry',
            parameters=[{
                'publish_tf': True,
                'publish_map_tf': False,
                'base_frame': 'base_link',     # Align with URDF
                'odometry_frame': 'odom',      # Standard Nav2 odometry frame
                'init_odom_with_first_pose': False
            }],
            remappings=[('odom', 'zed/odom')]
        )
    ])