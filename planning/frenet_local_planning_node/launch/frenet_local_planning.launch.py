"""Launch frenet_local_planning_node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('frenet_local_planning_node')
    default_config = os.path.join(
        pkg_share, 'config', 'frenet_local_planning.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to frenet_local_planning_node config YAML',
    )

    node = Node(
        package='frenet_local_planning_node',
        executable='frenet_local_planning_exe',
        name='frenet_local_planning_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([config_arg, node])
