"""Default is a preview with no hardware driver. Steering is always preview-only."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    stage = LaunchConfiguration('stage').perform(context)
    if stage not in ('dry-run', 'telemetry', 'drive'):
        raise ValueError('stage must be dry-run, telemetry or drive')
    config = os.path.join(get_package_share_directory('f1tenth_manual'),
                          'config', 'manual.yaml')
    profile = LaunchConfiguration('gamepad_profile').perform(context)
    if profile and not os.path.isfile(profile):
        raise ValueError('Gamepad profile does not exist: ' + profile)
    input_parameters = [config] + ([profile] if profile else [])
    actions = []
    if stage != 'telemetry':
        if LaunchConfiguration('gamepad').perform(context).lower() == 'true':
            actions.append(Node(package='joy', executable='joy_node', name='manual_joy',
                                parameters=input_parameters, remappings=[('joy', '/manual/joy')]))
        actions += [
            Node(package='joy_control_node', executable='joy_control_node_exe',
                 name='manual_control', parameters=input_parameters,
                 remappings=[('/joy', '/manual/joy'),
                             ('/ackermann_drive', '/manual/ackermann_drive'),
                             ('/ackermann_auto', '/manual/auto_unused')]),
            Node(package='vesc_ackermann', executable='ackermann_to_vesc_node',
                 namespace='vesc', name='ackermann_to_vesc', parameters=[config],
                 remappings=[('ackermann_cmd', '/manual/ackermann_drive'),
                             ('commands/motor/speed', '/manual/motor_speed' if
                              stage == 'drive' else '/manual/erpm_preview'),
                             ('commands/servo/position', '/manual/servo_preview')]),
        ]
    if stage != 'dry-run':
        routes = [(f'commands/motor/{mode}', f'/manual/disabled/{mode}')
                  for mode in ('duty_cycle', 'current', 'brake', 'position')]
        routes += [('commands/motor/speed', '/manual/motor_speed' if
                    stage == 'drive' else '/manual/disabled/speed'),
                   ('commands/servo/position', '/manual/disabled/servo')]
        actions.append(Node(package='vesc_driver', executable='vesc_driver_node',
                            namespace='vesc', name='vesc_driver',
                            parameters=[config], remappings=routes, output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('SDL_VIDEODRIVER', 'offscreen'),
        DeclareLaunchArgument('stage', default_value='dry-run'),
        DeclareLaunchArgument('gamepad', default_value='true'),
        DeclareLaunchArgument('gamepad_profile', default_value=''),
        OpaqueFunction(function=nodes),
    ])
