# pure_pursuit_node

`pure_pursuit_node` follows `/local_path/body` and publishes an autonomous Ackermann command.

The node is intentionally small and callback-driven:

- it does not subscribe to speed
- it does not reuse old local paths on a timer
- it computes one command immediately whenever a new `/local_path/body` message arrives

The speed command is limited from the preview path curvature:

```text
v = sqrt(lateral_accel_limit / curvature)
```

Then it is clamped by `min_speed` and `max_speed`.

## Pipeline Position

```text
track_path_publisher
  -> /path/center
  -> local_path_planner_node
  -> /local_path/body
  -> pure_pursuit_node
  -> /ackermann_auto
  -> joy_control_node
  -> /ackermann_drive
  -> arduino_mega_control_node
```

`joy_control_node` decides whether the auto command is passed through to `/ackermann_drive`.

## Inputs

| Topic | Type | Description |
| --- | --- | --- |
| `/local_path/body` | `nav_msgs/msg/Path` | Body-frame local path used for control. |
| `/local_path/global` | `nav_msgs/msg/Path` | Matching map/global path used only for RViz marker placement. |

Control is always computed from `/local_path/body`. The global path subscription exists so target
and curvature markers can be displayed in the RViz fixed frame, usually `map`, without requiring a
TF transform from body frame to map frame.

## Outputs

| Topic | Type | Description |
| --- | --- | --- |
| `/ackermann_auto` | `ackermann_msgs/msg/AckermannDriveStamped` | Autonomous speed and steering command. |
| `/pure_pursuit/target_point` | `visualization_msgs/msg/Marker` | Lookahead target point marker. |
| `/pure_pursuit/curvature_markers` | `visualization_msgs/msg/MarkerArray` | Curvature preview markers. |

The marker topics are enabled by `visualize_markers`.

## Pure Pursuit Steering

The node selects the first forward path point whose body-frame distance is at least
`lookahead_distance`.

Points with `x < min_forward_x` are ignored so the controller does not chase points behind or too
close to the vehicle.

The steering calculation is:

```text
Ld2 = target_x^2 + target_y^2
curvature = 2 * target_y / Ld2
steering_angle = atan(wheel_base * curvature)
```

The steering angle is clamped to:

```text
-max_steering_angle <= steering_angle <= max_steering_angle
```

## Curvature-Limited Speed

The node computes local curvature with three adjacent path points:

```text
prev, current, next
```

The preview window is limited by:

- `curvature_preview_distance`
- `curvature_preview_points`

For each sampled point, curvature is computed using the discrete derivative formula inherited from
the previous curvature-speed planner. The maximum curvature in the preview window is used for speed:

```text
speed = sqrt(lateral_accel_limit / max_curvature)
```

If curvature is below `curvature_epsilon`, the path is treated as straight enough and the command
uses `max_speed`.

## Visualization

When `visualize_markers` is true:

- the target point is drawn as a yellow sphere
- curvature preview points are drawn as spheres
- curvature color changes from green to red as curvature increases
- marker size also grows with curvature

If `/local_path/global` is available and has a valid frame, markers are published in that global
frame. Otherwise, the node falls back to the body-frame path frame.

This is important for RViz:

- RViz fixed frame is usually `map`
- `/local_path/body` may be `base_footprint`
- if no body-to-map TF exists, body-frame markers will not appear
- subscribing to `/local_path/global` lets markers appear on the map without TF

## Parameters

Configured in `config/pure_pursuit.yaml`.

| Parameter | Default | Description |
| --- | ---: | --- |
| `local_path_topic` | `/local_path/body` | Body-frame path used for control. |
| `global_path_topic` | `/local_path/global` | Map/global path used for marker placement. |
| `command_topic` | `/ackermann_auto` | Published autonomous Ackermann command. |
| `command_frame` | `base_footprint` | Fallback command frame. |
| `wheel_base` | `0.80` | Vehicle wheelbase in metres. |
| `lookahead_distance` | `2.0` | Target point distance in metres. |
| `max_steering_angle` | `0.524` | Steering angle limit in radians, about 30 degrees. |
| `max_speed` | `1.0` | Maximum command speed in m/s. |
| `min_speed` | `0.0` | Minimum command speed in m/s. |
| `lateral_accel_limit` | `0.25` | Lateral acceleration limit for curvature speed planning. |
| `curvature_epsilon` | `0.0001` | Curvature threshold treated as straight. |
| `curvature_preview_distance` | `2.5` | Max distance ahead used for curvature preview. |
| `curvature_preview_points` | `30` | Max number of preview points used for curvature. |
| `visualize_markers` | `true` | Enable target and curvature markers. |
| `target_marker_topic` | `/pure_pursuit/target_point` | Target marker topic. |
| `curvature_marker_topic` | `/pure_pursuit/curvature_markers` | Curvature marker topic. |
| `target_marker_scale` | `0.18` | Target marker sphere size. |
| `curvature_marker_scale` | `0.10` | Base curvature marker sphere size. |
| `curvature_marker_height` | `0.06` | Marker Z offset. |
| `max_visualized_curvature` | `2.0` | Curvature value mapped to full red marker color. |
| `min_forward_x` | `0.05` | Ignore path points behind or too close to the vehicle. |

## Dynamic Parameters

Most tuning parameters can be changed at runtime:

```bash
ros2 param set /pure_pursuit_node lookahead_distance 1.5
ros2 param set /pure_pursuit_node max_speed 0.8
ros2 param set /pure_pursuit_node lateral_accel_limit 0.2
ros2 param set /pure_pursuit_node visualize_markers true
```

Topic parameters require a node restart:

- `local_path_topic`
- `global_path_topic`
- `command_topic`
- `target_marker_topic`
- `curvature_marker_topic`

## Build

```bash
./build.sh --packages-select pure_pursuit_node
```

## Run

```bash
ros2 launch pure_pursuit_node pure_pursuit.launch.py
```

Override the config file:

```bash
ros2 launch pure_pursuit_node pure_pursuit.launch.py \
  config:=/path/to/pure_pursuit.yaml
```

## Quick Checks

Confirm local path inputs:

```bash
ros2 topic echo --once /local_path/body
ros2 topic echo --once /local_path/global
```

Confirm command output:

```bash
ros2 topic echo --once /ackermann_auto
```

Confirm markers:

```bash
ros2 topic echo --once /pure_pursuit/target_point
ros2 topic echo --once /pure_pursuit/curvature_markers
```

## RViz

`rviz/planner.rviz` already includes:

- `PurePursuit Target`
- `PurePursuit Curvature`

If markers do not appear:

1. Confirm `/local_path/global` is being published.
2. Confirm RViz fixed frame is `map`.
3. Confirm marker topics are enabled in RViz.
4. Confirm `visualize_markers` is true.

## Tuning Notes

- If steering is too aggressive, increase `lookahead_distance` or reduce `max_steering_angle`.
- If the vehicle cuts corners, decrease `lookahead_distance`.
- If speed is too high in curves, reduce `lateral_accel_limit` or `max_speed`.
- If speed drops too early, reduce `curvature_preview_distance`.
- If markers are all green, lower `max_visualized_curvature` to make curvature differences easier
  to see.

## Safety Notes

This node publishes only `/ackermann_auto`. The actual vehicle command still flows through
`joy_control_node` and then `/ackermann_drive`.

There is no stale-path timeout inside this node. It computes commands only when a fresh
`/local_path/body` message arrives. If the upstream local path stream stops, command freshness should
be handled by the downstream command multiplexer or vehicle interface.
