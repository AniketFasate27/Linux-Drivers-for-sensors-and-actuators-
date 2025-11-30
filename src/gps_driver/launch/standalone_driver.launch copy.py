from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='/dev/ttyUSB0')
    return LaunchDescription([
        port_arg,
        Node(
            package='gps_driver',
            executable='gps_driver_node',
            name='gps_driver',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': 4800,
                'timeout': 1.0,
                'serial_url': ''
            }],
            output='screen'
        )
    ])
