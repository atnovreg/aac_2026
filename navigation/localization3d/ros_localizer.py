"""Register deskewed FAST-LIO scans to a prior map and publish stamped base odometry."""
from pathlib import Path

import message_filters
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from .particle_localization import ParticleInitializer
from .streaming import MapSurfaces, MapTracker, imu_from_lidar, initial_pose, stamp_is_fresh


def transform_matrix(position, orientation):
    translation = [position.x, position.y, position.z]
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    if not np.isfinite(translation + quaternion).all() or abs(np.linalg.norm(quaternion)-1) > 1e-3:
        raise ValueError("Invalid pose or transform quaternion")
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    transform[:3, 3] = translation
    return transform


class IndoorLocalizer(Node):
    def __init__(self):
        super().__init__('indoor_localizer')
        map_dir = self.declare_parameter('map_dir', '').value
        self.global_initialize = self.declare_parameter('global_initialize', False).value
        if not self.global_initialize:
            self.declare_parameter('initial_base_pose', Parameter.Type.DOUBLE_ARRAY)
            self.seed = initial_pose(self.get_parameter('initial_base_pose').value)
        config = Path(__file__).resolve().parents[1] / 'fastlio/config/mid360_bag.yaml'
        config = self.declare_parameter('fastlio_config', str(config)).value
        self.imu_from_sensor = imu_from_lidar(config)
        self.max_age = self.declare_parameter('max_age_s', .5).value
        if not np.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('max_age_s must be positive')
        if not map_dir:
            raise ValueError('map_dir is required')
        self.surfaces = MapSurfaces(map_dir)
        self.initializer = (ParticleInitializer.from_map(map_dir, self.surfaces.centres)
                            if self.global_initialize else None)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tracker = None
        self.last_clock_ns = None
        self.replay = self.get_parameter('use_sim_time').value
        self.pending = None
        if self.replay:
            # A clock sample taken after ICP is needed to reject stale results.
            # Keep only the newest tick while the single executor processes a scan.
            clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Clock, '/clock', self.on_clock, clock_qos)
        self.output = self.create_publisher(Odometry, '/localization/odom', 5)
        self.odom = message_filters.Subscriber(
            self, Odometry, '/Odometry', qos_profile=qos_profile_sensor_data)
        self.clouds = message_filters.Subscriber(
            self, PointCloud2, '/cloud_registered_body', qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.TimeSynchronizer([self.odom, self.clouds], 8)
        self.sync.registerCallback(self.on_pair)
        self.get_logger().info("Prior loaded; waiting for matching FAST-LIO scans and odometry")

    def warn(self, text):
        self.get_logger().warning(text, throttle_duration_sec=2.0)

    def observe_clock(self, now_ns):
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self.pending = None
            if self.initializer is not None:
                self.initializer.reset()
                self.tracker = None
                self.warn('Clock rewound; restarting global localization')
            elif self.tracker is not None:
                self.tracker.requires_restart = True
                self.warn('Clock rewound; restart with a new seed')
        self.last_clock_ns = now_ns

    def on_clock(self, msg):
        now_ns = Time.from_msg(msg.clock).nanoseconds
        self.observe_clock(now_ns)
        if self.pending is not None:
            self.publish_pending(now_ns)

    def publish_pending(self, now_ns):
        stamp_ns, msg = self.pending
        self.pending = None
        age = (now_ns - stamp_ns) * 1e-9
        if self.tracker.requires_restart or not stamp_is_fresh(stamp_ns, now_ns, self.max_age):
            self.warn(f'ICP output age {age:.3f} s outside freshness window')
            return
        self.output.publish(msg)

    def on_pair(self, odom, cloud):
        if (odom.header.frame_id != 'camera_init' or odom.child_frame_id != 'body'
                or cloud.header.frame_id != odom.child_frame_id):
            self.warn('Unexpected FAST-LIO frame IDs')
            return
        stamp = Time.from_msg(odom.header.stamp)
        if not self.replay:
            self.observe_clock(self.get_clock().now().nanoseconds)
        now_ns = self.last_clock_ns or 0
        input_age = (now_ns - stamp.nanoseconds) * 1e-9
        if not stamp_is_fresh(stamp.nanoseconds, now_ns, self.max_age):
            self.warn(f'FAST-LIO input age {input_age:.3f} s outside freshness window')
            return
        try:
            pose = odom.pose.pose
            odom_from_imu = transform_matrix(pose.position, pose.orientation)
            points = point_cloud2.read_points_numpy(cloud, field_names=['x', 'y', 'z'],
                                                    skip_nans=True).reshape(-1, 3)
            if self.tracker is None:
                mount = self.buffer.lookup_transform('base_link', 'livox_frame', stamp).transform
                base_from_sensor = transform_matrix(mount.translation, mount.rotation)
                base_from_imu = base_from_sensor @ np.linalg.inv(self.imu_from_sensor)
                if self.initializer is None:
                    seed = self.seed
                else:
                    odom_from_base = odom_from_imu @ np.linalg.inv(base_from_imu)
                    base_points = points @ base_from_imu[:3, :3].T + base_from_imu[:3, 3]
                    seed, reason = self.initializer.update(odom_from_base, base_points)
                    if seed is None:
                        self.warn(reason)
                        return
                tracker = MapTracker(self.surfaces, seed, base_from_imu)
                state, reason = tracker.update(stamp.nanoseconds, odom_from_imu, points)
                if self.initializer is not None:
                    self.initializer.reset()
                    if not tracker.localized:
                        self.warn(f'Global hypothesis rejected by 3D ICP: {reason}')
                        return
                    self.get_logger().info('Global hypothesis confirmed by 3D ICP')
                self.tracker = tracker
            else:
                state, reason = self.tracker.update(stamp.nanoseconds, odom_from_imu, points)
                if self.tracker.requires_restart and self.initializer is not None:
                    self.initializer.reset()
                    self.pending = self.tracker = None
        except (TransformException, ValueError, np.linalg.LinAlgError) as exc:
            self.warn(str(exc))
            return
        if state is None:
            self.warn(reason)
            return
        map_from_base, linear, angular = state
        msg = Odometry()
        msg.header.stamp = odom.header.stamp
        msg.header.frame_id = 'indoor_map'
        msg.child_frame_id = 'base_link'
        dest = msg.pose.pose
        dest.position.x, dest.position.y, dest.position.z = map(float, map_from_base[:3, 3])
        quat = Rotation.from_matrix(map_from_base[:3, :3]).as_quat()
        dest.orientation.x, dest.orientation.y, dest.orientation.z, dest.orientation.w = map(float, quat)
        twist = msg.twist.twist
        twist.linear.x, twist.linear.y, twist.linear.z = map(float, linear)
        twist.angular.x, twist.angular.y, twist.angular.z = map(float, angular)
        self.pending = stamp.nanoseconds, msg
        if not self.replay:
            self.observe_clock(self.get_clock().now().nanoseconds)
            if self.pending is not None:
                self.publish_pending(self.last_clock_ns)


def main():
    rclpy.init()
    node = IndoorLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
