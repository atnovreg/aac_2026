# yolop_node

ROS2 Python wrapper for the vendored `perception/YOLOP` model.

YOLOP runs three driving-perception heads in one network:

- traffic-object detection
- drivable-area segmentation
- lane-line segmentation

This node is the recommended lane extraction path in the AAC perception stack.
It subscribes to a camera image topic and publishes lane and drivable-area masks
in ROS image format.

## Outputs

Default topics:

| Topic | Type | Encoding | Description |
| --- | --- | --- | --- |
| `/yolop/drivable_mask` | `sensor_msgs/Image` | `mono8` | Binary drivable-area mask. |
| `/yolop/lane_mask` | `sensor_msgs/Image` | `mono8` | Binary lane-line mask. |
| `/yolop/overlay` | `sensor_msgs/Image` | `bgr8` | Input image with drivable area, lane lines, and boxes overlaid. |
| `/yolop/stats` | `std_msgs/String` | JSON | Timing, mask ratios, detection count, device, and image size. |
| `/yolop/detections` | `std_msgs/String` | JSON | Detection list with class ID/name, confidence, and `xyxy` boxes. |

Input topic defaults to `/camera_front/image_raw`.

## YOLOP Source And Weights

The upstream YOLOP repository is vendored separately:

```text
/ros_ws/src/aac_2026/perception/YOLOP/
```

The default checkpoint is:

```text
/ros_ws/src/aac_2026/perception/YOLOP/weights/End-to-end.pth
```

`perception/YOLOP` is excluded from colcon with `COLCON_IGNORE`; only
`yolop_node` is built as a ROS2 package.

## Dependencies

Use the AAC Docker image's existing Torch/OpenCV/NumPy stack. Do not install the
full upstream YOLOP `requirements.txt`, because it can disturb the ROS Humble
`cv_bridge` NumPy/OpenCV ABI combination.

Root workspace requirements should include:

```text
prefetch_generator>=1.0.3
tensorboardX>=2.6
```

If using an older container, install only the missing helpers:

```bash
python3 -m pip install --no-cache-dir prefetch_generator tensorboardX
```

## Build

```bash
cd /ros_ws/src/aac_2026
./build.sh --packages-select yolop_node
source /ros_ws/install/setup.bash
```

## Run

Launch with the package config:

```bash
ros2 launch yolop_node yolop_node.launch.py
```

Run directly on GPU:

```bash
ros2 run yolop_node yolop_node --ros-args \
  -p yolop_dir:=/ros_ws/src/aac_2026/perception/YOLOP \
  -p weights_path:=/ros_ws/src/aac_2026/perception/YOLOP/weights/End-to-end.pth \
  -p image_topic:=/camera_front/image_raw \
  -p device:=cuda:0
```

Run on CPU for smoke testing:

```bash
ros2 run yolop_node yolop_node --ros-args -p device:=cpu
```

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `yolop_dir` | `/ros_ws/src/aac_2026/perception/YOLOP` | Vendored YOLOP repo path. |
| `weights_path` | `/ros_ws/src/aac_2026/perception/YOLOP/weights/End-to-end.pth` | YOLOP checkpoint path. |
| `image_topic` | `/camera_front/image_raw` | Input camera topic. |
| `drivable_mask_topic` | `/yolop/drivable_mask` | Drivable mask output topic. |
| `lane_mask_topic` | `/yolop/lane_mask` | Lane mask output topic. |
| `overlay_topic` | `/yolop/overlay` | Overlay output topic. |
| `stats_topic` | `/yolop/stats` | JSON stats output topic. |
| `detections_topic` | `/yolop/detections` | JSON detections output topic. |
| `device` | `"0"` | Inference device. Use `cuda:0`, `0`, or `cpu`. |
| `img_size` | `640` | YOLOP letterbox inference size. |
| `conf_thres` | `0.25` | Detection confidence threshold. |
| `iou_thres` | `0.45` | Detection NMS IoU threshold. |
| `overlay_alpha` | `0.55` | Drivable-area overlay strength. |
| `process_every_n` | `1` | Process every Nth input image. |
| `queue_size` | `1` | Input subscription queue size. |
| `reliable` | `false` | Use reliable QoS instead of best-effort. |
| `half_precision` | `true` | Use FP16 on CUDA. Ignored on CPU. |
| `publish_overlay` | `true` | Publish `/yolop/overlay`. |
| `publish_detections` | `true` | Publish `/yolop/detections`. |
| `draw_boxes` | `true` | Draw detections on overlay. |

## Runtime Checks

Verify topics:

```bash
ros2 topic list | grep yolop
```

Verify mask encodings:

```bash
ros2 topic echo --once /yolop/lane_mask --field encoding
ros2 topic echo --once /yolop/drivable_mask --field encoding
```

Expected encoding is `mono8`.

Inspect stats:

```bash
ros2 topic echo --once /yolop/stats
```

Typical stats fields:

```json
{
  "total_ms": 20.113,
  "inference_ms": 6.418,
  "nms_ms": 0.35,
  "drivable_ratio": 0.436012,
  "lane_ratio": 0.041808,
  "detections": 4,
  "device": "cuda:0",
  "img_size": 640
}
```

## Notes

- `yolop_node` keeps only the latest queued frame, so inference does not build a
  long backlog when the camera is faster than the model.
- YOLOP was trained for BDD100K-style driving scenes. For AAC camera geometry,
  lane masks may still benefit from ROI, BEV/IPM, or temporal post-processing.
- Upstream YOLOP demo changes and `tools/aac_smoke.py` are for offline testing;
  the ROS node does not depend on those scripts at runtime.
