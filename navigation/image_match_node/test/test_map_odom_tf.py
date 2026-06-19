"""Tests for global-pose to local-odometry TF conversion."""

import math

from geometry_msgs.msg import Pose

from localization_tf_node.map_odom_tf_node import _map_to_odom
import pytest


def _pose(x, y, yaw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)
    return pose


def test_map_to_odom_recovers_known_correction():
    """A known map-to-odom transform is recovered from two base poses."""
    expected_x = 1.25
    expected_y = -0.75
    expected_yaw = 0.35
    local_pose = _pose(4.0, -2.0, -0.4)

    cos_yaw = math.cos(expected_yaw)
    sin_yaw = math.sin(expected_yaw)
    global_pose = _pose(
        expected_x
        + cos_yaw * local_pose.position.x
        - sin_yaw * local_pose.position.y,
        expected_y
        + sin_yaw * local_pose.position.x
        + cos_yaw * local_pose.position.y,
        expected_yaw - 0.4,
    )

    x, y, yaw = _map_to_odom(global_pose, local_pose)

    assert x == pytest.approx(expected_x)
    assert y == pytest.approx(expected_y)
    assert yaw == pytest.approx(expected_yaw)
