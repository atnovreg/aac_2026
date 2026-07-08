# Parking Map Publisher

`parking_map_publisher` builds a static `nav_msgs/OccupancyGrid` for the parking mission.
It does not wait for SLAM Toolbox. Instead, it uses the exported map assets under
`planning/track_line_export`:

- `track_texture_lines_wide.csv`: road inner/outer boundaries
- `parking_lot_lines.csv`: parking-lot marking segments

The node marks the drivable road loop and parking-lot hull as free space, then overlays
road boundaries and selected parking lines as occupied cells. By default, RViz red,
orange, and yellow parking-line roles are not treated as obstacles:

- `parking_lot_boundary`
- `top_slot_header`
- `bottom_slot_edge`
- `entry_direction_marking`

This keeps the parallel-parking entry/start/finish markings traversable while still
letting Hybrid A* avoid slot separator markings.

The track `left_line` is the inner road boundary and is not drawn as an obstacle by
default. The outer `right_line` remains an obstacle so the planner does not leave the
known road/parking environment.

## Run

```bash
ros2 launch parking_map_publisher parking_map_publisher.launch.py use_sim_time:=true
```

The default output topic is `/parking_map` in the `map` frame. Hybrid A* is configured
to use this topic by default.

## Key Parameters

- `map_topic`: output occupancy topic, default `/parking_map`
- `resolution`: occupancy resolution in meters, default `0.05`
- `parking_line_obstacle_width`: occupied width drawn over parking markings
- `ignored_parking_line_roles`: parking CSV roles skipped by the obstacle overlay
- `road_boundary_obstacle_width`: occupied width drawn over road boundaries
- `mark_inner_road_boundary_obstacle`: draw track `left_line` as obstacle, default `false`
- `mark_outer_road_boundary_obstacle`: draw track `right_line` as obstacle, default `true`
- `parking_hull_margin`: free-space margin around the convex hull of parking markings
- `unknown_outside_drivable`: if `false`, cells outside known drivable regions are occupied
