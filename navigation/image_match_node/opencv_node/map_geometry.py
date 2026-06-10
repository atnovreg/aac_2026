"""Map image pixel and world coordinate conversion helpers."""

import math


def pixel_to_world(
        point, image_shape, x_bounds, y_bounds,
        swap_xy, flip_x, flip_y):
    """Convert a map image pixel into the configured world coordinates."""
    height, width = image_shape[:2]
    u_norm = float(point[0]) / max(1, width - 1)
    v_norm = float(point[1]) / max(1, height - 1)
    x_norm, y_norm = (
        (v_norm, u_norm) if swap_xy else (u_norm, v_norm))
    if flip_x:
        x_norm = 1.0 - x_norm
    if flip_y:
        y_norm = 1.0 - y_norm

    x = x_bounds[0] + x_norm * (x_bounds[1] - x_bounds[0])
    y = y_bounds[0] + y_norm * (y_bounds[1] - y_bounds[0])
    return x, y


def heading_from_pixels(
        center_point, forward_point, image_shape, x_bounds, y_bounds,
        swap_xy, flip_x, flip_y):
    """Convert two projected map pixels into a world-frame heading."""
    x, y = pixel_to_world(
        center_point,
        image_shape,
        x_bounds,
        y_bounds,
        swap_xy,
        flip_x,
        flip_y,
    )
    forward_x, forward_y = pixel_to_world(
        forward_point,
        image_shape,
        x_bounds,
        y_bounds,
        swap_xy,
        flip_x,
        flip_y,
    )
    return math.atan2(forward_y - y, forward_x - x)
