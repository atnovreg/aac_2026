# SAM3 ROS2 Image Topic Wrapper

This wrapper runs the upstream `perception/sam3` image API on a ROS2
`sensor_msgs/msg/Image` topic.

It does not modify the SAM3 submodule. Use the SAM3 virtual environment from
`../sam3/.venv-sam3`.

## Environment

Inside the x86 Docker container:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash || true

cd /ros_ws/src/aac_2026/perception/sam3_ros2
```

Pre-downloaded checkpoints are stored at:

```text
perception/sam3/checkpoints/
├── sam3.pt                 # SAM3 (~3.3 GB)
├── sam3.1_multiplex.pt     # SAM3.1 (~3.3 GB)
├── sam3_config.json
└── sam3.1_config.json
```

The wrapper automatically resolves the checkpoint in this order:

1. `--checkpoint <path>` if given explicitly
2. `perception/sam3/checkpoints/<hf-filename>` if the file exists (default)
3. HuggingFace local cache, then remote download (requires `--allow-download`)

## Smoke Test Without ROS Topic

The local checkpoint is picked up automatically (`perception/sam3/checkpoints/sam3.pt`):

```bash
../sam3/.venv-sam3/bin/python scripts/ros2_sam3_image_topic_wrapper.py \
  --image-file ../sam3/assets/images/truck.jpg \
  --prompt truck \
  --confidence-threshold 0.2 \
  --save-output-dir /tmp/sam3_ros2_smoke
```

To use SAM3.1 instead, pass `--hf-filename sam3.1_multiplex.pt`:

```bash
../sam3/.venv-sam3/bin/python scripts/ros2_sam3_image_topic_wrapper.py \
  --image-file ../sam3/assets/images/truck.jpg \
  --prompt truck \
  --confidence-threshold 0.2 \
  --hf-filename sam3.1_multiplex.pt \
  --save-output-dir /tmp/sam3_ros2_smoke
```

Expected result: one truck detection and debug images:

```bash
ls -lh /tmp/sam3_ros2_smoke
```

## Run On AAC Camera Topic

SAM3 (default):

```bash
../sam3/.venv-sam3/bin/python scripts/ros2_sam3_image_topic_wrapper.py \
  --image-topic /camera_front/image_raw \
  --prompt vehicle \
  --confidence-threshold 0.35 \
  --frame-stride 5 \
  --min-interval 0.5
```

SAM3.1 (faster multi-object tracking):

```bash
../sam3/.venv-sam3/bin/python scripts/ros2_sam3_image_topic_wrapper.py \
  --image-topic /camera_front/image_raw \
  --prompt vehicle \
  --confidence-threshold 0.35 \
  --frame-stride 5 \
  --min-interval 0.5 \
  --hf-filename sam3.1_multiplex.pt
```

For debug image saving:

```bash
../sam3/.venv-sam3/bin/python scripts/ros2_sam3_image_topic_wrapper.py \
  --image-topic /camera_front/image_raw \
  --prompt vehicle \
  --confidence-threshold 0.35 \
  --frame-stride 5 \
  --min-interval 0.5 \
  --save-output-dir /tmp/sam3_ros2_debug \
  --save-every 10
```

## Published Topics

- `/sam3/overlay`: `sensor_msgs/msg/Image`, `bgr8`, original image with masks and boxes.
- `/sam3/mask`: `sensor_msgs/msg/Image`, `mono8`, combined binary mask.
- `/sam3/detections`: `std_msgs/msg/String`, compact JSON with prompt, boxes, scores, and mask pixel counts.

Quick checks:

```bash
ros2 topic echo /sam3/detections
ros2 topic hz /sam3/overlay
```

In RViz2, add an Image display for `/sam3/overlay`.

## Notes

- Default QoS is best-effort with depth 1, which matches typical camera/sensor
  streams and drops stale frames instead of building a backlog.
- If a test publisher uses reliable QoS, run the wrapper with `--reliable`.
  For Gazebo camera topics, best-effort is usually the safer default.
- On the RTX 5080 test container, the `truck.jpg` smoke image runs at roughly
  0.38 seconds per inference after model load. Live camera streams should use
  `--frame-stride` or `--min-interval`.
- First startup loads a roughly 3.45 GB checkpoint and can take several seconds.
