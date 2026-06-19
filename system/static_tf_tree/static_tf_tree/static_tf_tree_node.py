"""Publish fixed AAC transforms from one YAML configuration."""

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml


def _quaternion_from_rpy(roll, pitch, yaw):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5
    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class StaticTfTreeNode(Node):
    """Load and publish all enabled fixed transforms."""

    def __init__(self):
        super().__init__('static_tf_tree')
        config_file = self.declare_parameter('config_file', '').value
        if not config_file:
            raise ValueError('config_file parameter must not be empty')

        with open(config_file, encoding='utf-8') as stream:
            config = yaml.safe_load(stream) or {}

        transforms = []
        for name, values in config.get('transforms', {}).items():
            if not values.get('enabled', True):
                self.get_logger().info(f'Skipping disabled transform: {name}')
                continue
            transforms.append(self._make_transform(name, values))

        if not transforms:
            raise ValueError('No enabled transforms found in config_file')

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f'Published {len(transforms)} fixed transforms from {config_file}'
        )

    def _make_transform(self, name, values):
        parent = values.get('parent')
        child = values.get('child')
        if not parent or not child:
            raise ValueError(
                f"Transform '{name}' requires parent and child frames")

        xyz = values.get('xyz', [0.0, 0.0, 0.0])
        rpy = values.get('rpy', [0.0, 0.0, 0.0])
        if len(xyz) != 3 or len(rpy) != 3:
            raise ValueError(
                f"Transform '{name}' requires three xyz and rpy values")

        quaternion = _quaternion_from_rpy(*map(float, rpy))
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(parent)
        transform.child_frame_id = str(child)
        transform.transform.translation.x = float(xyz[0])
        transform.transform.translation.y = float(xyz[1])
        transform.transform.translation.z = float(xyz[2])
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        self.get_logger().info(
            f'{name}: {parent} -> {child}, xyz={xyz}, rpy={rpy}')
        return transform


def main(args=None):
    """Run the centralized static TF publisher."""
    rclpy.init(args=args)
    node = StaticTfTreeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
