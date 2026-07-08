"""Launch the Hybrid A* global planner."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hybrid_a_star_planner_node')
    default_config = os.path.join(
        pkg_share, 'config', 'hybrid_a_star_planner.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to hybrid_a_star_planner config YAML',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )
    start_pose_source_arg = DeclareLaunchArgument(
        'start_pose_source',
        default_value='tf',
        description='Start pose source: tf, initialpose, or auto',
    )
    use_occupancy_grid_arg = DeclareLaunchArgument(
        'use_occupancy_grid',
        default_value='true',
        description='Use an occupancy grid when it is available',
    )
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/parking_map',
        description='OccupancyGrid topic for Hybrid A* collision checking',
    )
    reverse_enabled_arg = DeclareLaunchArgument(
        'reverse_enabled',
        default_value='true',
        description='Allow reverse search primitives for parking maneuvers',
    )
    analytic_expansion_type_arg = DeclareLaunchArgument(
        'analytic_expansion_type',
        default_value='reed_shepp',
        description='Analytic expansion type: none, straight, dubins, reed_shepp, or auto',
    )

    node = Node(
        package='hybrid_a_star_planner_node',
        executable='hybrid_a_star_planner_exe',
        name='hybrid_a_star_planner_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'start_pose_source': LaunchConfiguration('start_pose_source')},
            {'use_occupancy_grid': LaunchConfiguration('use_occupancy_grid')},
            {'map_topic': LaunchConfiguration('map_topic')},
            {'reverse_enabled': LaunchConfiguration('reverse_enabled')},
            {'analytic_expansion_type': LaunchConfiguration('analytic_expansion_type')},
        ],
    )

    return LaunchDescription([
        config_arg,
        use_sim_time_arg,
        start_pose_source_arg,
        use_occupancy_grid_arg,
        map_topic_arg,
        reverse_enabled_arg,
        analytic_expansion_type_arg,
        node,
    ])
