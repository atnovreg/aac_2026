# IPM BEV Node

전방 및 후방 카메라 영상과 차량의 카메라 TF를 이용해 지면 평면의
Inverse Perspective Mapping(IPM) 이미지를 생성하는 ROS2 C++ 노드입니다.

## Coordinate Convention

기본 설정은 AAC Gazebo 차량에 맞춰져 있습니다.

```text
base_footprint:
  +X forward
  +Y left
  +Z up

BEV image:
  top    = forward
  left   = vehicle left
  bottom = rear
```

현재 카메라 장착 위치는
`system/aac_vehicle/config/sensor_config.yaml`에서 읽혀
`robot_state_publisher`가 다음 TF를 제공합니다.

```text
base_footprint -> base_link -> camera_front_link
base_footprint -> base_link -> camera_rear_link
```

노드는 각 BEV 픽셀을 `base_footprint` 지면의 3D 점으로 변환한 뒤,
TF와 CameraInfo를 사용해 전후방 카메라 영상에 역투영합니다.

## Published Topics

| Topic | Encoding | Description |
|---|---|---|
| `/ipm_bev/front` | `bgr8` | 전방 카메라 IPM |
| `/ipm_bev/rear` | `bgr8` | 후방 카메라 IPM |
| `/ipm_bev/fused` | `bgr8` | 전후방 합성 BEV |
| `/ipm_bev/valid_mask` | `mono8` | 카메라가 관측 가능한 영역 |
| `/ipm_bev/source_mask` | `mono8` | `100=front`, `200=rear` |
| `/ipm_bev/debug` | `bgr8` | 1m grid와 차량 footprint가 표시된 BEV |

## Build

Docker 컨테이너 안에서 실행합니다.

```bash
cd /ros_ws
source /opt/ros/$ROS_DISTRO/setup.bash
src/aac_2026/perception/setup_colcon_ignores.sh

colcon build \
  --base-paths /ros_ws/src/aac_2026/perception/ipm_bev_node \
  --symlink-install \
  --packages-select ipm_bev_node

source install/setup.bash
```

## Run With AAC Stack

The normal team workflow is to run this node through the repository-level
tmux session:

```bash
cd /ros_ws/src/aac_2026
./build.sh
./start.sh
```

`start.sh` loads `operations/tmuxp_config/config.yaml`, which currently starts
Gazebo vehicle spawning, joystick control, and:

```bash
ros2 launch ipm_bev_node deploy.launch.py
```

Stop the stack cleanly with:

```bash
cd /ros_ws/src/aac_2026
./stop.sh
```

## Run Standalone

Gazebo 차량과 `robot_state_publisher`를 먼저 실행한 뒤:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /ros_ws/install/setup.bash

ros2 launch ipm_bev_node deploy.launch.py
```

확인:

```bash
rqt_image_view /ipm_bev/debug
ros2 topic hz /ipm_bev/fused
ros2 topic echo /ipm_bev/fused --once --field header
```

기본 설정의 출력 크기는 `480 x 800`이며 현재 Gazebo 환경에서 약
`9-13 Hz`로 동작하는 것을 확인했습니다.

## CameraInfo Fallback

현재 Gazebo bridge는 동일한 intrinsic을 사용하는 전후방 카메라에 대해
공용 `/camera_info`를 제공합니다. CameraInfo가 도착하지 않을 경우에는
다음 설정으로 intrinsic을 계산합니다.

```yaml
allow_fov_fallback: true
horizontal_fov: 1.2217
```

실차에서는 전후방 CameraInfo topic을 분리하고 실제 calibration 결과를
사용하는 것이 좋습니다.

## Main Parameters

```yaml
base_frame: base_footprint
front_camera_frame: camera_front_link
rear_camera_frame: camera_rear_link
camera_frame_convention: link_x_forward

x_min: -10.0
x_max: 10.0
y_min: -6.0
y_max: 6.0
resolution: 0.025
ground_z: 0.0
```

`camera_frame_convention`:

- `link_x_forward`: AAC Gazebo camera link, X forward/Y left/Z up
- `ros_optical`: ROS optical frame, X right/Y down/Z forward

## Assumptions

- 도로를 `base_frame`의 `ground_z` 평면으로 가정합니다.
- 경사로, 요철, 차량 pitch/roll 변화가 크면 IPM 오차가 증가합니다.
- 전후방 영상의 timestamp 차이가 `sync_tolerance_sec`보다 크면 해당
  frame pair는 처리하지 않습니다.
