"""Launch pure_pursuit_node."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pure_pursuit_node')
    default_config = os.path.join(pkg_share, 'config', 'pure_pursuit.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to pure_pursuit_node config YAML',
    )

    node = Node(
        package='pure_pursuit_node',
        executable='pure_pursuit_exe',
        name='pure_pursuit_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([config_arg, node])
