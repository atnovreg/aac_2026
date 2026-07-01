# road_seg_node

ROS2 Python node for realtime road/drivable-area segmentation with Ultralytics
YOLO checkpoints.

This node is useful for quick experiments with Ultralytics semantic
segmentation checkpoints such as `yolo26n-sem.pt`, and for smoke-testing
instance segmentation checkpoints in `task:=segment` mode. For lane-line
extraction, prefer `yolop_node` unless you have a custom Ultralytics checkpoint
trained with lane classes.

## Outputs

Default topics:

| Topic | Type | Encoding | Description |
| --- | --- | --- | --- |
| `/road_seg/mask` | `sensor_msgs/Image` | `mono8` | Binary mask for selected road/drivable classes. |
| `/road_seg/overlay` | `sensor_msgs/Image` | `bgr8` | Input image with selected pixels overlaid in green. |
| `/road_seg/class_map` | `sensor_msgs/Image` | `mono8` | Raw semantic class-id map. Only published in `task:=semantic`. |
| `/road_seg/stats` | `std_msgs/String` | JSON | Timing, pixel counts, ratios, device, and image size. |

Input topic defaults to `/camera_front/image_raw`.

## Models

Project-local Ultralytics checkpoints are stored in:

```text
/ros_ws/src/aac_2026/models/
  yolo26n-sem.pt
  yolo26n-seg.pt
```

Default config uses:

```text
/ros_ws/src/aac_2026/models/yolo26n-sem.pt
```

`yolo26n-sem.pt` is a semantic segmentation model. `yolo26n-seg.pt` is a COCO
instance segmentation model and does not include road or lane classes.

## Dependencies

The rebuilt AAC Docker image installs the root workspace dependencies,
including `ultralytics`.

If using an older container, install the package-specific runtime dependencies:

```bash
cd /ros_ws/src/aac_2026
python3 -m pip install --no-cache-dir -r perception/road_seg_node/requirements.txt
```

The package requirements pin `numpy==1.26.4` and `opencv-python<4.12` to avoid
breaking ROS Humble `cv_bridge` with NumPy 2.x ABI changes.

## Build

```bash
cd /ros_ws/src/aac_2026
./build.sh --packages-select road_seg_node
source /ros_ws/install/setup.bash
```

## Run

Launch with the package config:

```bash
ros2 launch road_seg_node road_seg_node.launch.py
```

Run directly on CPU:

```bash
ros2 run road_seg_node road_seg_node --ros-args \
  -p task:=semantic \
  -p model_path:=/ros_ws/src/aac_2026/models/yolo26n-sem.pt \
  -p image_topic:=/camera_front/image_raw \
  -p device:=cpu \
  -p imgsz:=1024
```

Run on GPU:

```bash
ros2 run road_seg_node road_seg_node --ros-args \
  -p task:=semantic \
  -p model_path:=/ros_ws/src/aac_2026/models/yolo26n-sem.pt \
  -p device:=cuda:0
```

## Segment Mode

Instance segmentation mode merges masks from selected class names or class IDs
into `/road_seg/mask`.

Smoke-test the COCO segment checkpoint:

```bash
ros2 run road_seg_node road_seg_node --ros-args \
  -p task:=segment \
  -p model_path:=/ros_ws/src/aac_2026/models/yolo26n-seg.pt \
  -p road_class_names:="car,bicycle,truck,traffic light" \
  -p device:=cuda:0
```

For lane extraction with Ultralytics, use a custom checkpoint trained with lane
classes:

```bash
ros2 run road_seg_node road_seg_node --ros-args \
  -p task:=segment \
  -p model_path:=/path/to/lane-seg.pt \
  -p road_class_names:=lane,lane_marking \
  -p device:=cuda:0
```

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `task` | `semantic` | YOLO task, `semantic` or `segment`. |
| `model_path` | `/ros_ws/src/aac_2026/models/yolo26n-sem.pt` | Checkpoint path. |
| `image_topic` | `/camera_front/image_raw` | Input camera topic. |
| `mask_topic` | `/road_seg/mask` | Binary output mask topic. |
| `overlay_topic` | `/road_seg/overlay` | Overlay output topic. |
| `class_map_topic` | `/road_seg/class_map` | Semantic class-map output topic. |
| `stats_topic` | `/road_seg/stats` | JSON stats topic. |
| `device` | `cuda` in config, `cpu` in code fallback | Inference device. Use `cpu` or `cuda:0`. |
| `imgsz` | `1024` | Ultralytics inference image size. |
| `road_class_names` | `road` | Comma-separated class names to extract. |
| `road_class_ids` | empty | Optional comma-separated class IDs. Overrides names. |
| `overlay_alpha` | `0.45` | Green overlay strength. |
| `process_every_n` | `1` | Process every Nth image. |
| `publish_class_map` | `true` | Publish semantic class map. |
| `publish_stats` | `true` | Publish JSON stats. |

## Checks

Verify topics:

```bash
ros2 topic list | grep road_seg
ros2 topic echo --once /road_seg/stats
```

Verify mask encoding:

```bash
ros2 topic echo --once /road_seg/mask --field encoding
```

Expected encoding is `mono8`.

## Notes

- `yolo26n-seg.pt` is not a lane detector; it is included only for segment-mode
  smoke tests.
- For lane-line output and drivable-area output in one pass, use `yolop_node`.
- If `cv_bridge` fails with NumPy ABI errors, check that NumPy is `1.26.4` in
  the active container Python environment.
