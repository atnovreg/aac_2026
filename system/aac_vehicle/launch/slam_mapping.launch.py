import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aac_vehicle')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    input_odom_topic = LaunchConfiguration('input_odom_topic')
    input_scan_topic = LaunchConfiguration('input_scan_topic')
    output_scan_topic = LaunchConfiguration('output_scan_topic')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time; set false on the real vehicle',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                pkg_share, 'config', 'slam_mapper_params.yaml'),
            description='AAC SLAM Toolbox parameter file',
        ),
        DeclareLaunchArgument(
            'input_odom_topic',
            default_value='/odom',
            description='Odometry input from Gazebo or the real vehicle',
        ),
        DeclareLaunchArgument(
            'input_scan_topic',
            default_value='/scan',
            description='LaserScan input from Gazebo or the real vehicle',
        ),
        DeclareLaunchArgument(
            'output_scan_topic',
            default_value='/scan_lidar',
            description='Isolated LaserScan topic consumed by SLAM Toolbox',
        ),
        DeclareLaunchArgument('odom_frame', default_value='odom_lidar'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_footprint_lidar'),
        DeclareLaunchArgument(
            'lidar_frame', default_value='lidar_link_lidar'),
        Node(
            package='aac_vehicle',
            executable='odom_tf_publisher',
            name='lidar_odom_tf_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': input_odom_topic,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
            }],
            output='screen',
        ),
        Node(
            package='aac_vehicle',
            executable='scan_frame_relay',
            name='lidar_scan_frame_relay',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': input_scan_topic,
                'output_topic': output_scan_topic,
                'output_frame': lidar_frame,
            }],
            output='screen',
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox_lidar',
            parameters=[
                slam_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'odom_frame': odom_frame,
                    'base_frame': base_frame,
                    'scan_topic': output_scan_topic,
                },
            ],
            output='screen',
        ),
    ])
