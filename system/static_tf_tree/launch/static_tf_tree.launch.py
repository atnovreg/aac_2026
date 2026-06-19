"""Launch the centralized AAC static TF publisher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create the static TF tree launch description."""
    package_share = get_package_share_directory('static_tf_tree')
    default_config = os.path.join(
        package_share, 'config', 'static_tf_tree.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Static transform YAML file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        Node(
            package='static_tf_tree',
            executable='static_tf_tree_node',
            name='static_tf_tree',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
