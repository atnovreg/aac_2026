# Semantic Segmentation ROS2 Wrapper

Common ROS2 wrapper for comparing lightweight neural semantic segmentation
models on AAC Gazebo camera topics.

The wrapper is model-agnostic. It keeps the ROS2 interface, latest-frame
dropping, debug outputs, and timing metrics common, while each model lives in a
small backend adapter.

Supported backends:

- `onnxruntime`: exported ONNX semantic segmentation models
- `pidnet_s`: PIDNet-S via the official PIDNet PyTorch repo
- `bisenetv2`: BiSeNetV2 via the CoinCheung/BiSeNet PyTorch repo
- `torchscript`: exported TorchScript segmentation model

## Why One Wrapper

PIDNet-S and BiSeNetV2 have different model constructors, preprocessing, and
checkpoint formats. They can still share the same ROS2 wrapper because their
runtime contract is the same:

```text
BGR image -> backend.infer() -> HxW class-id label image
```

The wrapper then publishes comparable outputs:

- `/semantic_seg/labels`: `mono8` class-id image
- `/semantic_seg/drivable`: `mono8` binary mask
- `/semantic_seg/lane_markings`: `mono8` binary mask
- `/semantic_seg/overlay`: `bgr8` visualization
- `/semantic_seg/stats`: JSON timing and pixel-count stats

## Model Sources

The wrapper uses external model code through git submodules.

```bash
cd /ros_ws/src/aac_2026
git submodule update --init --recursive perception/PIDNet perception/BiSeNet
```

Download checkpoints into each repo or into a shared checkpoint folder, then
edit:

```text
configs/pidnet_s_gazebo.yaml
configs/bisenetv2_gazebo.yaml
```

In the current AAC Docker workflow, use the helper first:

```bash
cd /ros_ws/src/aac_2026/perception/semantic_seg_ros2

# Needed for the default PIDNet-S ONNX config if your image was built before
# docker/dockerfiles/x86.Dockerfile included onnxruntime-gpu.
python3 -m pip install --user onnxruntime-gpu==1.23.2

python3 scripts/download_semantic_seg_assets.py --all
```

This downloads:

- Qualcomm PIDNet ONNX export into
  `/ros_ws/src/aac_2026/perception/PIDNet/pretrained_models/qualcomm_pidnet_onnx_float/`
- BiSeNetV2 Cityscapes checkpoint into
  `/ros_ws/src/aac_2026/perception/BiSeNet/res/model_final_v2_city.pth`

The original PIDNet PyTorch checkpoint is still available only through the
official PIDNet Google Drive links. If you manually download it, put it here and
use `configs/pidnet_s_pytorch_gazebo.yaml`:

```text
/ros_ws/src/aac_2026/perception/PIDNet/pretrained_models/cityscapes/PIDNet_S_Cityscapes_test.pt
```

## Smoke Test Without Checkpoints

Use these commands first to check that Python imports, CUDA, and the wrapper
path are working. The output masks are meaningless because the models use
random weights when checkpoints are missing.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash || true

cd /ros_ws/src/aac_2026/perception/semantic_seg_ros2

python3 scripts/ros2_semantic_seg_wrapper.py \
  --config configs/pidnet_s_smoke_random.yaml \
  --image-file /ros_ws/src/aac_2026/system/simple_gazebo_track/models/simple_track/materials/textures/track.png \
  --save-output-dir /tmp/aac_pidnet_smoke

python3 scripts/ros2_semantic_seg_wrapper.py \
  --config configs/bisenetv2_smoke_random.yaml \
  --image-file /ros_ws/src/aac_2026/system/simple_gazebo_track/models/simple_track/materials/textures/track.png \
  --save-output-dir /tmp/aac_bisenetv2_smoke
```

## Run PIDNet-S

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash || true

cd /ros_ws/src/aac_2026/perception/semantic_seg_ros2

python3 scripts/ros2_semantic_seg_wrapper.py \
  --config configs/pidnet_s_gazebo.yaml \
  --image-topic /camera_front/image_raw \
  --frame-stride 1 \
  --save-output-dir /tmp/aac_pidnet_debug \
  --save-every 30
```

`configs/pidnet_s_gazebo.yaml` uses the Qualcomm PIDNet ONNX export because it
is downloadable from Docker without browser interaction. For the original
official PyTorch `.pt` weight, use `configs/pidnet_s_pytorch_gazebo.yaml` after
placing the checkpoint manually.

## Run BiSeNetV2

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash || true

cd /ros_ws/src/aac_2026/perception/semantic_seg_ros2

python3 scripts/ros2_semantic_seg_wrapper.py \
  --config configs/bisenetv2_gazebo.yaml \
  --image-topic /camera_front/image_raw \
  --frame-stride 1 \
  --save-output-dir /tmp/aac_bisenetv2_debug \
  --save-every 30
```

## One-Shot Image Test

This requires the assets from `scripts/download_semantic_seg_assets.py`.

```bash
python3 scripts/ros2_semantic_seg_wrapper.py \
  --config configs/pidnet_s_gazebo.yaml \
  --image-file /ros_ws/src/aac_2026/system/simple_gazebo_track/models/simple_track/materials/textures/track.png \
  --save-output-dir /tmp/aac_pidnet_smoke
```

## Comparing Models

Run both models with the same:

- input topic
- `input_size`
- class mapping
- saved debug interval
- camera scene

Then compare `/semantic_seg/stats`:

```bash
ros2 topic echo /semantic_seg/stats
ros2 topic hz /semantic_seg/overlay
```

For side-by-side comparisons, run one model at a time or override output topics:

```bash
python3 scripts/ros2_semantic_seg_wrapper.py \
  --config configs/pidnet_s_gazebo.yaml \
  --overlay-topic /pidnet/overlay \
  --stats-topic /pidnet/stats

python3 scripts/ros2_semantic_seg_wrapper.py \
  --config configs/bisenetv2_gazebo.yaml \
  --overlay-topic /bisenetv2/overlay \
  --stats-topic /bisenetv2/stats
```

## Class Mapping Notes

Cityscapes pretrained checkpoints use class `0` for `road`, but they do not
have a separate lane-marking class. For lane-aware AAC driving masks, train a
custom 3-class model:

```text
0 background
1 drivable
2 lane_marking
```

Use `configs/custom_3class_template.yaml` as a starting point:

```yaml
output:
  drivable_class_ids: [1, 2]
  lane_class_ids: [2]
```

The existing `road_mask_ros2` traditional image-processing scripts can produce
Gazebo pseudo-labels for this first custom training pass.

## Real-Time Behavior

The ROS callback only stores the latest image. Inference runs in a worker
thread. If the model is slower than the camera, old frames are dropped and the
latest frame is processed. This keeps latency bounded during driving tests.
