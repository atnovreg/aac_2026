"""Launch a single speed-mode VESC output path with manual priority."""
from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    repo = Path(__file__).resolve().parents[4]

    def nodes(context):
        hardware = LaunchConfiguration('hardware').perform(context)
        if hardware not in ('true', 'false'):
            raise ValueError('hardware must be true or false')
        manual = LaunchConfiguration('manual_config').perform(context)
        config = LaunchConfiguration('calibration_file').perform(context)
        clock = {'use_sim_time': False}
        actions = [
            Node(package='joy', executable='joy_node', name='manual_joy',
                 parameters=[manual, clock], remappings=[('joy', '/manual/joy')], on_exit=Shutdown()),
            Node(package='joy_control_node', executable='joy_control_node_exe', name='manual_control',
                 parameters=[manual, clock, {'manual_only': True}],
                 remappings=[('/joy', '/manual/joy'), ('/ackermann_drive', '/manual/ackermann_drive'),
                             ('/ackermann_auto', '/manual/auto_unused')], on_exit=Shutdown()),
            ExecuteProcess(cmd=[sys.executable, '-m', 'system.f1tenth.ros_drive_selector',
                                '--ros-args', '-p', f'manual_config:={manual}',
                                '-p', f'calibration_file:={config}', '-p', f'hardware:={hardware}',
                                '-p', 'use_sim_time:=false'], cwd=str(repo), output='screen',
                           on_exit=Shutdown()),
        ]
        if hardware == 'true':
            actions.append(Node(
                package='vesc_driver', executable='vesc_driver_node', namespace='vesc', name='vesc_driver',
                parameters=[manual, clock, {f'{mode}_{bound}': 0.
                            for mode in ('current', 'brake', 'duty_cycle', 'position')
                            for bound in ('min', 'max')}],
                remappings=[(f'commands/motor/{mode}', f'/drive/disabled/{mode}')
                            for mode in ('current', 'brake', 'duty_cycle', 'position')],
                output='screen', on_exit=Shutdown(),
            ))
        return actions

    return LaunchDescription([
        DeclareLaunchArgument('manual_config'),
        DeclareLaunchArgument('calibration_file', default_value=str(Path(__file__).parent / 'indoor_drive.yaml')),
        DeclareLaunchArgument('hardware', default_value='false'),
        SetEnvironmentVariable('SDL_VIDEODRIVER', 'offscreen'),
        OpaqueFunction(function=nodes),
    ])
