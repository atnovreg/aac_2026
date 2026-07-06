# frenet_local_planning_node

ROS 2 node that converts the body-frame local path into Frenet-style local
motion primitives.

## Overview

Input:

- `/local_path/body` (`nav_msgs/msg/Path`)
- `/odom_gt` (`nav_msgs/msg/Odometry`, visualization only)

Outputs:

- `/frenet/motion_primitives` (`aac_msgs/msg/PathWithCostArray`)
- `/frenet/optimal_path` (`nav_msgs/msg/Path`)
- `/frenet/candidate_cloud` (`sensor_msgs/msg/PointCloud2`)
- `/frenet/reference_boundary` (`geometry_msgs/msg/PolygonStamped`)
- `/frenet/map_visualization/optimal_path` (`nav_msgs/msg/Path`)
- `/frenet/map_visualization/candidate_cloud` (`sensor_msgs/msg/PointCloud2`)
- `/frenet/map_visualization/reference_boundary` (`geometry_msgs/msg/PolygonStamped`)

The node samples several parallel reference paths around the incoming local
path, generates lateral Frenet candidates for each reference, assigns lightweight
cost values, and publishes the minimum-cost candidate. Obstacle/costmap terms
are not included yet; `collision_cost` is set to `0.0`.

The `map_visualization` topics are RViz-only outputs. They transform the
body-frame candidate coordinates into the configured visualization frame using
the latest odometry pose, so they appear in front of the vehicle on the map.
After the first valid visualization is produced, the node republishes the last
valid map visualization if a later cycle has a temporary missing/invalid input.

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
- `parallel_path_count`: number of parallel reference paths
- `parallel_scale`: lateral span for parallel references [m]
- `left_width_margin`, `right_width_margin`, `width_delta`: Frenet lateral samples [m]
- `planning_speed_mps`: speed used for longitudinal candidate sampling [m/s]
- `publish_map_visualization`: enable fixed-frame RViz-only debug topics
- `visualization_frame_id`: frame id used by the RViz-only debug topics

## Build

```bash
./build.sh --packages-select aac_msgs frenet_local_planning_node
```
