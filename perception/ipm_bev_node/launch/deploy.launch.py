import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('ipm_bev_node'),
        'config',
        'ipm_bev.yaml',
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='IPM BEV parameter file',
    )

    node = Node(
        package='ipm_bev_node',
        executable='ipm_bev_node_exe',
        name='ipm_bev_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([config_arg, node])
