# launch/standalone_driver.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port = LaunchConfiguration('port')
    serial_url = LaunchConfiguration('serial_url')
    baud = LaunchConfiguration('baud')
    timeout = LaunchConfiguration('timeout')
    exec_name = LaunchConfiguration('exec_name')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_url', default_value=''),        # e.g., tcp://127.0.0.1:9000 or a pty
        DeclareLaunchArgument('baud', default_value='4800'),
        DeclareLaunchArgument('timeout', default_value='1.0'),
        # Set this to the console_script you install in setup.py (see below)
        DeclareLaunchArgument('exec_name', default_value='driver'),

        Node(
            package='gps_driver',
            executable=exec_name,
            #executable='driver',
            name='gps_driver',
            output='screen',
            parameters=[{
                'port': port,
                'serial_url': serial_url,
                'baud': baud,
                'timeout': timeout,
            }],
        ),
    ])
