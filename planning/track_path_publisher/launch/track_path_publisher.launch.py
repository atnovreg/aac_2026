"""Launch track_path_publisher — publishes left/center/right lane paths."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('track_path_publisher')
    default_config = os.path.join(pkg_share, 'config', 'track_path_publisher.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to track_path_publisher config YAML',
    )

    node = Node(
        package='track_path_publisher',
        executable='track_path_publisher_exe',
        name='track_path_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([config_arg, node])
