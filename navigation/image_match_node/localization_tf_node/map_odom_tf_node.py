"""Publish map to odom from a global pose estimate and local odometry."""

from collections import deque
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def _stamp_nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def _map_to_odom(global_pose, local_pose):
    map_base_yaw = _yaw_from_quaternion(global_pose.orientation)
    odom_base_yaw = _yaw_from_quaternion(local_pose.orientation)
    map_odom_yaw = math.atan2(
        math.sin(map_base_yaw - odom_base_yaw),
        math.cos(map_base_yaw - odom_base_yaw),
    )

    cos_yaw = math.cos(map_odom_yaw)
    sin_yaw = math.sin(map_odom_yaw)
    rotated_odom_x = (
        cos_yaw * local_pose.position.x
        - sin_yaw * local_pose.position.y
    )
    rotated_odom_y = (
        sin_yaw * local_pose.position.x
        + cos_yaw * local_pose.position.y
    )

    return (
        global_pose.position.x - rotated_odom_x,
        global_pose.position.y - rotated_odom_y,
        map_odom_yaw,
    )


class MapOdomTfNode(Node):
    """Convert global base pose estimates into a map to odom correction."""

    def __init__(self):
        super().__init__('map_odom_tf_publisher')

        self.declare_parameter(
            'global_odometry_topic',
            '/visual_localization/opencv/odometry',
        )
        self.declare_parameter('local_odometry_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('max_odom_age_sec', 0.25)
        self.declare_parameter('odom_buffer_size', 300)
        self.declare_parameter('publish_rate', 20.0)

        self._global_topic = self.get_parameter(
            'global_odometry_topic').value
        self._local_topic = self.get_parameter(
            'local_odometry_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._max_odom_age_ns = int(
            self.get_parameter('max_odom_age_sec').value * 1.0e9)
        buffer_size = self.get_parameter('odom_buffer_size').value
        self._odom_buffer = deque(maxlen=int(buffer_size))
        self._latest_transform = None

        self._broadcaster = TransformBroadcaster(self)
        self._local_subscription = self.create_subscription(
            Odometry,
            self._local_topic,
            self._local_odom_callback,
            qos_profile_sensor_data,
        )
        self._global_subscription = self.create_subscription(
            Odometry,
            self._global_topic,
            self._global_odom_callback,
            10,
        )
        publish_rate = self.get_parameter('publish_rate').value
        self._publish_timer = self.create_timer(
            1.0 / float(publish_rate),
            self._publish_latest_transform,
        )

        self.get_logger().info(
            f'Building {self._map_frame} -> {self._odom_frame} from '
            f'{self._global_topic} and {self._local_topic}'
        )

    def _local_odom_callback(self, message):
        self._odom_buffer.append(message)

    def _global_odom_callback(self, message):
        self.get_logger().debug('Received global odometry estimate.')
        if not self._odom_buffer:
            self.get_logger().warning(
                'Waiting for local odometry.',
                throttle_duration_sec=5.0,
            )
            return

        target_stamp = _stamp_nanoseconds(message.header.stamp)
        local_odom = min(
            self._odom_buffer,
            key=lambda odom: abs(
                _stamp_nanoseconds(odom.header.stamp) - target_stamp
            ),
        )
        odom_age_ns = abs(
            _stamp_nanoseconds(local_odom.header.stamp) - target_stamp
        )
        if odom_age_ns > self._max_odom_age_ns:
            self.get_logger().warning(
                'Rejected global pose: nearest odometry is '
                f'{odom_age_ns / 1.0e9:.3f} s away.',
                throttle_duration_sec=2.0,
            )
            return

        x, y, yaw = _map_to_odom(
            message.pose.pose,
            local_odom.pose.pose,
        )
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw * 0.5)
        transform.transform.rotation.w = math.cos(yaw * 0.5)
        self._latest_transform = transform
        self._broadcaster.sendTransform(self._latest_transform)
        self.get_logger().debug(
            f'Updated {self._map_frame} -> {self._odom_frame}: '
            f'x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
        )

    def _publish_latest_transform(self):
        if self._latest_transform is None:
            return

        self._latest_transform.header.stamp = self.get_clock().now().to_msg()
        self._broadcaster.sendTransform(self._latest_transform)


def main(args=None):
    """Run the map to odom TF publisher."""
    rclpy.init(args=args)
    node = MapOdomTfNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
