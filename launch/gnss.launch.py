from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# `ros2 launch IGVC_2025 gnss.launch.py connection_type:=tcp port:="192.168.0.222:55555"``
# or
# `ros2 launch IGVC_2025 gnss.launch.py port:="/dev/ttyACMX" baudrate:=115200``

def generate_launch_description():
    return LaunchDescription([
        # Configuration parameters
        DeclareLaunchArgument(
            'connection_type',
            default_value='serial',
            description='Type of connection: serial or tcp'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port device or TCP address (host:port)'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baud rate for serial connection'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='gps',
            description='TF frame ID for GPS messages'
        ),
        
        # Swift Navigation ROS 2 Driver Node
        Node(
            package='swiftnav_ros2',
            executable='swiftnav_driver',
            name='swiftnav_driver',
            output='screen',
            parameters=[{
                'connection_type': LaunchConfiguration('connection_type'),
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'frame_name': LaunchConfiguration('frame_id'),
                'timestamp_source_gnss': True,  # Use GNSS timestamps :cite[1]
                
                # Message enablement
                'publish_gpsfix': True,
                'publish_navsatfix': True,
                'publish_twist': False,  # Disable if not needed
                'publish_baseline': False,  # Disable if not needed
                
                # Advanced settings :cite[1]
                'track_update_min_speed_mps': 0.5,
                'dops_timeout': 2.0
            }]
        )
    ])