from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='4800'),
        Node(
            package='rtk_driver',
            executable='rtk_driver',
            name='rtk_driver',
            parameters=[{'port': port, 'baud': baud}],
            output='screen'
        )
    ])
