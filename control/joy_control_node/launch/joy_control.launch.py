import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('joy_control_node')

    joy_id_arg = DeclareLaunchArgument(
        'joy_id', default_value='0',
        description='Joystick device index (0 = /dev/input/js0)')

    joy_id = LaunchConfiguration('joy_id')

    # joy_node: joystick driver → /joy (sensor_msgs/Joy)
    # SDL_VIDEODRIVER=offscreen: ROS2 Humble joy는 SDL2 기반으로,
    # headless(디스플레이 없음) 환경에서 이 변수 없이는 joystick 감지 불가
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': joy_id,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
            'coalesce_interval_ms': 1,
        }],
        additional_env={'SDL_VIDEODRIVER': 'offscreen'},
        output='screen'
    )

    # joy_control_node: /joy → /ackermann_drive
    joy_control = Node(
        package='joy_control_node',
        executable='joy_control_node_exe',
        name='joy_control_node',
        parameters=[os.path.join(pkg_share, 'config', 'joy_control_params.yaml')],
        output='screen'
    )

    return LaunchDescription([
        joy_id_arg,
        joy_node,
        joy_control,
    ])
