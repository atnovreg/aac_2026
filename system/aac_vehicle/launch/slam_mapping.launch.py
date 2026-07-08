import os
import math
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wait_for_initial_odom(topic, timeout_sec):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data

    logger = logging.get_logger('slam_mapping.launch')
    received = {'msg': None}

    rclpy_context = rclpy.context.Context()
    rclpy.init(args=[], context=rclpy_context)
    node = rclpy.create_node(
        'slam_mapping_initial_odom_waiter', context=rclpy_context)
    executor = SingleThreadedExecutor(context=rclpy_context)
    executor.add_node(node)

    def on_odom(msg):
        received['msg'] = msg

    node.create_subscription(Odometry, topic, on_odom, qos_profile_sensor_data)
    logger.info(
        f"Waiting for initial odometry on '{topic}' before starting SLAM Toolbox"
        + (f" (timeout={timeout_sec:.1f}s)" if timeout_sec > 0.0 else ""))

    start = time.monotonic()
    try:
        while rclpy.ok(context=rclpy_context) and received['msg'] is None:
            if timeout_sec > 0.0 and (time.monotonic() - start) >= timeout_sec:
                break
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=rclpy_context)

    msg = received['msg']
    if msg is None:
        logger.warning(
            f"No odometry received on '{topic}' before timeout; "
            "SLAM Toolbox will start without initial odometry pose override.")
        return None

    pose = msg.pose.pose
    yaw = _yaw_from_quaternion(pose.orientation)
    logger.info(
        "Initial odometry pose captured for SLAM startup "
        f"frame='{msg.header.frame_id}': "
        f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, yaw={yaw:.3f} rad")
    return [float(pose.position.x), float(pose.position.y), float(yaw)]


def _slam_launch_setup(context, *args, **kwargs):
    use_sim_time_value = LaunchConfiguration('use_sim_time').perform(context)
    slam_params_file = LaunchConfiguration('slam_params_file').perform(context)
    input_odom_topic = LaunchConfiguration('input_odom_topic').perform(context)
    output_scan_topic = LaunchConfiguration('output_scan_topic').perform(context)
    odom_frame = LaunchConfiguration('odom_frame').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    wait_for_initial_odom = _as_bool(
        LaunchConfiguration('wait_for_initial_odom').perform(context))
    initial_odom_timeout = float(
        LaunchConfiguration('initial_odom_timeout').perform(context))

    use_sim_time = _as_bool(use_sim_time_value)
    initial_pose = None
    if wait_for_initial_odom:
        initial_pose = _wait_for_initial_odom(input_odom_topic, initial_odom_timeout)

    slam_overrides = {
        'use_sim_time': use_sim_time,
        'odom_frame': odom_frame,
        'base_frame': base_frame,
        'scan_topic': output_scan_topic,
    }
    if initial_pose is not None:
        slam_overrides['map_start_pose'] = initial_pose
        slam_overrides['map_start_at_dock'] = False

    return [
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox_lidar',
            parameters=[
                slam_params_file,
                slam_overrides,
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('aac_vehicle')
    use_sim_time = LaunchConfiguration('use_sim_time')
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
            default_value='/odom_gt',
            description='Odometry input. Gazebo should use /odom_gt; set /odom on the real vehicle.',
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
        DeclareLaunchArgument(
            'wait_for_initial_odom',
            default_value='true',
            description='Wait for the first odometry message before starting SLAM Toolbox',
        ),
        DeclareLaunchArgument(
            'initial_odom_timeout',
            default_value='0.0',
            description='Seconds to wait for initial odometry; 0 waits forever',
        ),
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
        TimerAction(
            period=0.2,
            actions=[OpaqueFunction(function=_slam_launch_setup)],
        ),
    ])
