"""Tests for map image and Gazebo world coordinate alignment."""

import math

import pytest

from opencv_node.map_geometry import (
    heading_from_pixels,
    pixel_to_world,
)


IMAGE_SHAPE = (2292, 3054)
X_BOUNDS = (-6.0, 6.0)
Y_BOUNDS = (-8.0, 8.0)


def test_flip_y_corrects_x_axis_mirror():
    """Gazebo world Y is opposite to the texture image U direction."""
    point = (2200.0, 500.0)
    x_old, y_old = pixel_to_world(
        point, IMAGE_SHAPE, X_BOUNDS, Y_BOUNDS, True, True, False)
    x_new, y_new = pixel_to_world(
        point, IMAGE_SHAPE, X_BOUNDS, Y_BOUNDS, True, True, True)

    assert x_new == pytest.approx(x_old)
    assert y_new == pytest.approx(-y_old)


def test_flip_y_corrects_mirrored_heading():
    """The same Y-axis correction also reverses the mirrored yaw."""
    center = (1800.0, 1300.0)
    forward = (1700.0, 1100.0)
    yaw_old = heading_from_pixels(
        center, forward, IMAGE_SHAPE, X_BOUNDS, Y_BOUNDS,
        True, True, False)
    yaw_new = heading_from_pixels(
        center, forward, IMAGE_SHAPE, X_BOUNDS, Y_BOUNDS,
        True, True, True)

    assert math.sin(yaw_new) == pytest.approx(-math.sin(yaw_old))
    assert math.cos(yaw_new) == pytest.approx(math.cos(yaw_old))
