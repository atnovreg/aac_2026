"""Launch parking_lines_publisher — publishes parking-lot line markers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('parking_lines_publisher')
    default_config = os.path.join(pkg_share, 'config', 'parking_lines_publisher.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to parking_lines_publisher config YAML',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    node = Node(
        package='parking_lines_publisher',
        executable='parking_lines_publisher_exe',
        name='parking_lines_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([config_arg, use_sim_time_arg, node])
