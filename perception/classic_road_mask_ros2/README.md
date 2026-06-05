# Road Mask ROS2 Utilities

OpenCV-based utilities for the AAC Gazebo track.

This is a traditional image-processing approach, not a neural segmentation
model. It uses HSV color thresholding, morphological filtering, connected
component cleanup, and simple ROI masking to extract drivable-road and
lane-marking masks from the controlled Gazebo camera/texture images.

The current `simple_track.world` renders the track as one texture:

```text
system/simple_gazebo_track/models/simple_track/materials/textures/track.png
```

These scripts provide a lightweight first pass before using a neural
segmentation model:

- drivable road area: dark asphalt plus lane paint
- lane markings: white/yellowish paint
- overlay: original image with masks blended in

## 1. Extract Masks From The Static Track Texture

Inside the Docker container:

```bash
cd /ros_ws/src/aac_2026/perception/classic_road_mask_ros2

python3 scripts/extract_track_texture_masks.py \
  --output-dir /tmp/aac_track_masks
```

Outputs:

- `drivable_mask.png`
- `road_surface_mask.png`
- `lane_markings_mask.png`
- `overlay.png`
- `metadata.json`

`metadata.json` includes the simple pixel-to-world mapping for the `12 x 16 m`
track visual.

## 2. Run From ROS2 Camera Image Topic

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash || true

cd /ros_ws/src/aac_2026/perception/classic_road_mask_ros2

python3 scripts/ros2_road_lane_mask_wrapper.py \
  --image-topic /camera_front/image_raw \
  --frame-stride 2 \
  --roi-y-min-ratio 0.25
```

Published topics:

- `/road_mask/drivable`: `sensor_msgs/msg/Image`, `mono8`
- `/road_mask/lane_markings`: `sensor_msgs/msg/Image`, `mono8`
- `/road_mask/overlay`: `sensor_msgs/msg/Image`, `bgr8`
- `/road_mask/stats`: `std_msgs/msg/String`, compact JSON

Quick checks:

```bash
ros2 topic echo /road_mask/stats
ros2 topic hz /road_mask/overlay
```

In RViz2, add an Image display for `/road_mask/overlay`.

## 3. Tuning

The defaults are tuned for the current Gazebo `track.png`.

Road/asphalt threshold:

```bash
--road-hsv-lower 0,0,25
--road-hsv-upper 179,90,125
```

Lane paint threshold:

```bash
--lane-hsv-lower 0,0,165
--lane-hsv-upper 179,80,255
```

If the mask includes too much wall/sky/vehicle body, increase the lower ROI:

```bash
--roi-y-min-ratio 0.35
```

If camera images come from a reliable QoS publisher, add:

```bash
--reliable
```

## 4. One-Shot Image Smoke Test

```bash
python3 scripts/ros2_road_lane_mask_wrapper.py \
  --image-file /ros_ws/src/aac_2026/system/simple_gazebo_track/models/simple_track/materials/textures/track.png \
  --roi-y-min-ratio 0.0 \
  --save-output-dir /tmp/aac_road_mask_smoke
```

This writes:

- `smoke_drivable.png`
- `smoke_lane.png`
- `smoke_overlay.png`
