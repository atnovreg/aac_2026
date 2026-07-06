# stanley_node

`stanley_node` is a ROS 2 Stanley controller for the existing AAC local planning stack.
It subscribes to the body-frame local path and publishes an autonomous Ackermann command.

## Interface

Subscriptions:

- `/local_path/body` (`nav_msgs/msg/Path`): local path in the vehicle body frame. The vehicle is assumed to be at `(0, 0)` with yaw `0`.

Publications:

- `/ackermann_auto` (`ackermann_msgs/msg/AckermannDriveStamped`): autonomous steering and speed command consumed by `joy_control_node`.
- `/stanley/error_markers` (`visualization_msgs/msg/MarkerArray`): nearest path segment, projected point, heading arrow, and cross-track error line for RViz.

## Control Logic

For every incoming local path message, the node:

1. Finds the closest valid path segment to the vehicle origin.
2. Projects the vehicle origin onto that segment.
3. Computes signed cross-track error. Positive error means the path is on the left side of the vehicle.
4. Computes heading error between the body x-axis and the selected path segment direction.
5. Applies PID to heading error and cross-track error.
6. Computes steering with:

```text
steering = heading_pid(heading_error)
         + atan2(stanley_gain * cte_pid(cross_track_error),
                 abs(speed) + softening_speed)
```

The final steering angle is clamped by `max_steering_angle`.

## Run

```bash
ros2 launch stanley_node stanley.launch.py
```

With a custom config:

```bash
ros2 launch stanley_node stanley.launch.py config:=/path/to/stanley.yaml
```

## Key Parameters

- `local_path_topic`: input body-frame path topic.
- `command_topic`: output Ackermann command topic.
- `stanley_gain`: gain applied to the Stanley cross-track correction.
- `softening_speed`: denominator offset that prevents unstable correction near zero speed.
- `cte_kp`, `cte_ki`, `cte_kd`: PID gains for cross-track error.
- `heading_kp`, `heading_ki`, `heading_kd`: PID gains for heading error.
- `target_speed`, `max_speed`, `min_speed`: speed command limits.
- `cte_speed_reduction_gain`, `heading_speed_reduction_gain`: optional speed reduction from tracking error.
- `min_segment_forward_x`: ignores path segments that are completely behind this body-frame x value.

## Tuning Notes

Start with only proportional control:

```yaml
cte_kp: 1.0
cte_ki: 0.0
cte_kd: 0.0
heading_kp: 1.0
heading_ki: 0.0
heading_kd: 0.0
stanley_gain: 1.0
```

If the vehicle cuts corners or reacts slowly, increase `stanley_gain` or `cte_kp`.
If the vehicle oscillates around the path, reduce `stanley_gain`, reduce `cte_kp`, or add a small `cte_kd`.
Use integral terms only after the sign and proportional gains are confirmed.
