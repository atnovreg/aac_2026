"""Launch stanley_node."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('stanley_node')
    default_config = os.path.join(pkg_share, 'config', 'stanley.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to stanley_node config YAML',
    )

    node = Node(
        package='stanley_node',
        executable='stanley_exe',
        name='stanley_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([config_arg, node])
