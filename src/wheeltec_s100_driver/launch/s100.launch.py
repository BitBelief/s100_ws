import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'wheeltec_s100_driver'


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory(PKG), 'config', 's100.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        Node(
            package=PKG,
            executable='s100_driver',
            name='s100_driver',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'port': LaunchConfiguration('port')},
            ],
        ),
    ])
