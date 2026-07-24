# frenet_local_planning_node

ROS 2 node that converts the body-frame local path into Frenet-style local
motion primitives.

## Overview

Input:

- `/local_path/body` (`nav_msgs/msg/Path`)
- `/odom_gt` (`nav_msgs/msg/Odometry`, visualization only)
- `/scan` (`sensor_msgs/msg/LaserScan`, obstacle scoring)

Outputs:

- `/frenet/motion_primitives` (`aac_msgs/msg/PathWithCostArray`)
- `/frenet/optimal_path` (`nav_msgs/msg/Path`)
- `/frenet/candidate_cloud` (`sensor_msgs/msg/PointCloud2`)
- `/frenet/reference_boundary` (`geometry_msgs/msg/PolygonStamped`)
- `/frenet/map_visualization/optimal_path` (`nav_msgs/msg/Path`)
- `/frenet/map_visualization/candidate_cloud` (`sensor_msgs/msg/PointCloud2`)
- `/frenet/map_visualization/reference_boundary` (`geometry_msgs/msg/PolygonStamped`)

The node samples Frenet lateral candidates around the incoming local path,
transforms LaserScan returns into the path frame, and publishes the lowest-cost
collision-free candidate. `collision_cost` is a continuous Gaussian potential:

```text
exp(-clearance^2 / (2 * obstacle_psi^2))
```

Candidates inside `robot_collision_radius` are rejected. If every candidate is
blocked, the node publishes an empty optimal path so the downstream controller
stops instead of following a stale path.

The `map_visualization` topics are RViz-only outputs. They transform the
body-frame candidate coordinates into the configured visualization frame using
the latest odometry pose, so they appear in front of the vehicle on the map.
The last valid map visualization is kept by transient-local QoS. Cached
republishing is disabled by default so stale map coordinates do not appear as
fresh data when a later cycle has a temporary missing/invalid input.

## Run

```bash
ros2 launch frenet_local_planning_node frenet_local_planning.launch.py
```

The default configuration is installed from:

```text
config/frenet_local_planning.yaml
```

Key parameters:

- `local_path_topic`: input local path, default `/local_path/body`
- `odom_topic`: vehicle pose used for map visualization, default `/odom_gt`
- `scan_topic`: obstacle LaserScan, default `/scan`
- `obstacle_psi`: width of the smooth obstacle potential [m]
- `obstacle_influence_distance`: clearance beyond which obstacle cost is zero [m]
- `robot_collision_radius`: circular robot footprint used for rejection [m]
- `obstacle_cost_weight`: obstacle potential weight in the total path score
- `transient_cost_weight`: discourages rapid left/right candidate switching
- `parallel_path_count`: number of parallel reference paths
- `parallel_scale`: lateral span for parallel references [m]
- `left_width_margin`, `right_width_margin`, `width_delta`: Frenet lateral samples [m]
- `planning_speed_mps`: speed used for longitudinal candidate sampling [m/s]
- `publish_map_visualization`: enable fixed-frame RViz-only debug topics
- `republish_cached_map_visualization`: republish cached map debug topics on invalid cycles
- `visualization_frame_id`: frame id used by the RViz-only debug topics

## Build

```bash
./build.sh --packages-select aac_msgs frenet_local_planning_node
```
