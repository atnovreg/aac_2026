# local_path_planner_node

`local_path_planner_node` extracts a short local path from the global centerline and publishes it in
both map/global frame and body frame.

It is the bridge between the static track path and the controller:

```text
/path/center + odometry
  -> local_path_planner_node
  -> /local_path/global
  -> /local_path/body
  -> pure_pursuit_node
```

The node does not use TF lookup. It uses the subscribed odometry pose directly, so the odometry pose
must already be expressed in the same global frame as `/path/center`.

## Inputs

| Topic | Type | QoS | Description |
| --- | --- | --- | --- |
| `/path/center` | `nav_msgs/msg/Path` | transient local, depth 1 | Static global centerline from `track_path_publisher`. |
| `/odom_gt` | `nav_msgs/msg/Odometry` | sensor data | Vehicle pose in the path/global frame. |

The odometry topic is configurable. The default config uses:

```yaml
odom_topic: "/odom_gt"
```

For the real vehicle, change it to the estimated odometry topic, for example:

```yaml
odom_topic: "/odom_estimated"
```

## Outputs

| Topic | Type | Frame | Description |
| --- | --- | --- | --- |
| `/local_path/global` | `nav_msgs/msg/Path` | path frame, usually `map` | Lookahead path segment in global coordinates. |
| `/local_path/body` | `nav_msgs/msg/Path` | odom `child_frame_id` or `body_frame` | Same path transformed into vehicle body coordinates. |

`/local_path/body` is the controller input. `/local_path/global` is useful for RViz and for
map-frame controller visualization markers.

## Closest Waypoint Selection

At every timer tick, the node finds the closest global path waypoint using a combined cost:

```text
cost = distance + heading_error_weight * abs(shortest_heading_error)
```

This is more stable than distance-only matching on loop tracks, because two track segments can be
geometrically close but face opposite directions.

`heading_error_weight` has units of metres per radian. Larger values make heading agreement more
important when selecting the closest waypoint.

## Local Path Extraction

After the closest waypoint is found, the node copies waypoints forward along the global path until
the accumulated arc length reaches `lookahead_dist`.

The path wraps around, so closed-loop tracks continue smoothly across the last waypoint back to the
first waypoint.

For each selected waypoint, the node publishes:

- a global-frame pose copied from the global path
- a body-frame pose computed from the current odometry pose

The body transform is:

```text
dx = path_x - ego_x
dy = path_y - ego_y

body_x =  cos(yaw) * dx + sin(yaw) * dy
body_y = -sin(yaw) * dx + cos(yaw) * dy
```

## Parameters

Configured in `config/local_path_planner.yaml`.

| Parameter | Default | Description |
| --- | ---: | --- |
| `lookahead_dist` | `5.0` | Local path length to extract forward from closest waypoint, in metres. |
| `heading_error_weight` | `0.5` | Weight for heading mismatch in closest waypoint cost. |
| `update_rate` | `100.0` | Timer rate in Hz. |
| `global_frame` | `map` | Fallback global frame when messages do not provide one. |
| `body_frame` | `base_footprint` | Fallback body frame when odometry `child_frame_id` is empty. |
| `odom_topic` | `/odom_gt` | Odometry input topic. |
| `global_path_topic` | `/path/center` | Global center path input topic. |
| `local_global_topic` | `/local_path/global` | Output local path in global frame. |
| `local_body_topic` | `/local_path/body` | Output local path in body frame. |

## Build

```bash
./build.sh --packages-select local_path_planner_node
```

## Run

```bash
ros2 launch local_path_planner_node local_path_planner.launch.py
```

Override the odometry topic:

```bash
ros2 launch local_path_planner_node local_path_planner.launch.py \
  --ros-args -p odom_topic:=/odom_estimated
```

Override the config file:

```bash
ros2 launch local_path_planner_node local_path_planner.launch.py \
  config:=/path/to/local_path_planner.yaml
```

## Quick Checks

Confirm global path is available:

```bash
ros2 topic echo --once /path/center
```

Confirm odometry is available:

```bash
ros2 topic echo --once /odom_gt
```

Confirm local paths:

```bash
ros2 topic echo --once /local_path/global
ros2 topic echo --once /local_path/body
```

## Frame Assumptions

This node intentionally does not transform odometry with TF. If the odometry frame and global path
frame differ, the node logs a warning:

```text
Odometry frame '...' differs from global path frame '...'.
No TF transform is applied; odom pose is assumed to already be in the path frame.
```

Fix this upstream by publishing odometry in the same frame as `/path/center`.

## Tuning Notes

- If the local path jumps to a nearby but wrong part of the loop, increase `heading_error_weight`.
- If pure pursuit reacts too late, reduce `lookahead_dist` or tune pure pursuit `lookahead_distance`.
- If output is noisy, check odometry yaw stability first. Body-frame path quality depends directly
  on odometry pose and yaw.

## Typical Pipeline

Start the upstream path publisher first:

```bash
ros2 launch track_path_publisher track_path_publisher.launch.py
```

Then run this node:

```bash
ros2 launch local_path_planner_node local_path_planner.launch.py
```

Then run the controller:

```bash
ros2 launch pure_pursuit_node pure_pursuit.launch.py
```
