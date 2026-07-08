# Parking Lines Publisher

Publishes parking-lot marking line segments extracted from
`planning/track_line_export/parking_lot_lines.csv` as RViz markers.

## Run

```bash
ros2 launch parking_lines_publisher parking_lines_publisher.launch.py use_sim_time:=true
```

## Topics

- Output: `/parking_lines/markers` (`visualization_msgs/msg/MarkerArray`)

## Parameters

- `csv_path`: CSV file or directory. Empty uses the installed `parking_lot_lines.csv`.
- `marker_topic`: marker output topic.
- `frame_id`: output frame, default `map`.
- `publish_rate`: publish rate, default `1.0` Hz.
- `line_width`, `z_offset`: visualization styling.
- `publish_labels`: publish text labels with `line_id:line_role`.

The CSV must contain world-coordinate endpoint columns:
`x1_m`, `y1_m`, `x2_m`, `y2_m`, plus `line_id`, `line_type`,
`line_role`, `orientation`, and `length_m`.
