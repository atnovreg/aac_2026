# track_path_publisher

`track_path_publisher` converts the CSV exported by `planning/track_line_export` into three static
`nav_msgs/msg/Path` topics:

- `/path/left`
- `/path/center`
- `/path/right`

In the current track convention, `left` means the inner road boundary, `right` means the outer road
boundary, and `center` is the midpoint line. The node is intended to be the first planning-stage
publisher in the path pipeline:

```text
track_line_export CSV
  -> track_path_publisher
  -> /path/center
  -> local_path_planner_node
  -> /local_path/{global,body}
  -> pure_pursuit_node
```

## Inputs

The node reads one wide-format CSV file. It does not subscribe to ROS topics.

The CSV must contain these millimetre-coordinate columns:

```text
left_x_mm,left_y_mm
center_x_mm,center_y_mm
right_x_mm,right_y_mm
```

The parser reads columns by header name, not by fixed column index. This makes it robust to column
ordering changes in future `track_line_export` outputs.

## CSV Resolution

The `csv_path` parameter accepts either:

- empty string: use the installed default directory
- a direct CSV file path
- a directory containing CSV files

When a directory is provided, the node recursively finds CSV files and prefers filenames containing
`_wide.csv`. If multiple candidates are found, it uses the lexicographically first candidate and logs
a warning.

The default installed directory is:

```text
share/track_path_publisher/data/track_line_export
```

During CMake install, the sibling source directory is copied as a folder:

```text
planning/track_line_export/
  -> share/track_path_publisher/data/track_line_export/
```

This avoids hard-coding a specific CSV filename such as `track_lines_wide.csv`.

## Outputs

| Topic | Type | QoS | Description |
| --- | --- | --- | --- |
| `/path/left` | `nav_msgs/msg/Path` | transient local, depth 1 | Inner boundary path |
| `/path/center` | `nav_msgs/msg/Path` | transient local, depth 1 | Centerline path used by local planner |
| `/path/right` | `nav_msgs/msg/Path` | transient local, depth 1 | Outer boundary path |

The transient local QoS lets late subscribers receive the last published path immediately.

Each output path is rebuilt once at startup, then republished periodically with a fresh timestamp.

## Coordinate Transform

The CSV is exported in image/drawing coordinates and millimetres. The node maps those coordinates
into the configured world bounds:

```text
map_x_min <= x <= map_x_max
map_y_min <= y <= map_y_max
```

The current config mirrors the image matching convention:

```yaml
map_swap_xy: true
map_flip_x:  true
map_flip_y:  true
scale_width_mm:  16000.0
scale_height_mm: 11000.0
```

These values must match the `track_line_export` scale arguments and the map convention used by
localization/image matching. If the path appears mirrored in RViz, check `map_swap_xy`,
`map_flip_x`, and `map_flip_y` first.

## Parameters

Configured in `config/track_path_publisher.yaml`.

| Parameter | Default | Description |
| --- | ---: | --- |
| `csv_path` | `""` | CSV file or export directory. Empty uses installed `data/track_line_export`. |
| `waypoint_sampling_dist` | `0.2` | Arc-length spacing between resampled waypoints in metres. |
| `publish_rate` | `1.0` | Republish rate in Hz. |
| `frame_id` | `map` | Frame ID for all three output paths. |
| `map_x_min`, `map_x_max` | `-5.5`, `5.5` | World X bounds. |
| `map_y_min`, `map_y_max` | `-8.0`, `8.0` | World Y bounds. |
| `map_swap_xy` | `true` | Swap normalized drawing X/Y before mapping. |
| `map_flip_x` | `true` | Flip normalized world X. |
| `map_flip_y` | `true` | Flip normalized world Y. |
| `scale_width_mm` | `16000.0` | Export drawing width in mm. |
| `scale_height_mm` | `11000.0` | Export drawing height in mm. |

## Build

From the AAC repository root inside the container:

```bash
./build.sh --packages-select track_path_publisher
```

## Run

```bash
ros2 launch track_path_publisher track_path_publisher.launch.py
```

Override the config file:

```bash
ros2 launch track_path_publisher track_path_publisher.launch.py \
  config:=/path/to/track_path_publisher.yaml
```

Override only the CSV path:

```bash
ros2 launch track_path_publisher track_path_publisher.launch.py \
  --ros-args -p csv_path:=/ros_ws/src/aac_2026/planning/track_line_export
```

## Quick Checks

List topics:

```bash
ros2 topic list | grep '^/path/'
```

Check one message:

```bash
ros2 topic echo --once /path/center
```

Check path sizes from node log. A normal startup log looks like:

```text
Resolved csv_path directory ... to CSV .../track_texture_lines_wide.csv
Loaded paths - left: ... center: ... right: ... waypoints
```

## Troubleshooting

### `No CSV files found`

Check `csv_path`. If it is empty, rebuild/install the package so that
`share/track_path_publisher/data/track_line_export` exists.

### `Missing required CSV column`

The CSV is not the expected wide format. Regenerate it with `track_line_export` and confirm the
`left_x_mm`, `center_x_mm`, and `right_x_mm` column groups exist.

### Path is mirrored or rotated

Review the map transform parameters:

- `map_swap_xy`
- `map_flip_x`
- `map_flip_y`
- `map_x_min`, `map_x_max`
- `map_y_min`, `map_y_max`

### Local planner does not receive `/path/center`

The publisher uses transient local QoS, but the subscriber must be compatible. The current
`local_path_planner_node` subscribes with transient local QoS for `/path/center`.
