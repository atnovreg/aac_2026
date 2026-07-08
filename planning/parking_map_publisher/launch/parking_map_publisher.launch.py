"""Launch the static parking mission occupancy map publisher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('parking_map_publisher')
    default_config = os.path.join(pkg_share, 'config', 'parking_map_publisher.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to parking_map_publisher config YAML',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/parking_map',
        description='Output OccupancyGrid topic',
    )

    node = Node(
        package='parking_map_publisher',
        executable='parking_map_publisher_exe',
        name='parking_map_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'map_topic': LaunchConfiguration('map_topic')},
        ],
    )

    return LaunchDescription([
        config_arg,
        use_sim_time_arg,
        map_topic_arg,
        node,
    ])
