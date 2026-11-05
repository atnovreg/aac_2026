# AAC 자율주행 개발

앞으로 수정할 코드는 `~/ros2_ws/src/aac_2026`에 있습니다. 본체와 Orin Nano의 F1TENTH 코드를 기존 AAC의 센서·위치 추정·계획·제어 분류에 통합했습니다.

| 기존 분류 | 추가한 작업 |
|---|---|
| `sensors/` | MID-360·RealSense 드라이버, 전방 필터, bag 어댑터 |
| `navigation/` | FAST-LIO, 지도 필터·검토, 오프라인 3D 위치 추정 |
| `planning/` | 건지광장 경로 작업, track 생성 |
| `control/` | MPCC, C++ 가상 차량, 튜너·분석 |
| `system/` | F1TENTH 장비 설정·공통 경로 함수, Gazebo 궤적 재생 |
| `docker/` | F1TENTH Docker 환경·Livox SDK 소스 |
| `operations/` | 실행·설치 보조, 검증, 설명서·과거 작업 색인 |

[작업 위치·실행 방법](operations/f1tenth/docs/F1TENTH_시작하기.md) · [장비 설정·AGX 자료 목록](system/f1tenth/README_KO.md) · [과거 작업 보관 위치](operations/f1tenth/archive/README_KO.md)

## 새 장비에 소스 받기

비공개 저장소는 `atnovreg/aac_2026`이며 원본 `Leedk3/aac_2026`의 커밋 이력과 서브모듈 출처를 유지합니다.

```bash
gh auth login
mkdir -p ~/ros2_ws/src
gh repo clone atnovreg/aac_2026 ~/ros2_ws/src/aac_2026 -- --recurse-submodules
cd ~/ros2_ws/src/aac_2026
python3 operations/f1tenth/setup_workspace.py
bash operations/f1tenth/run.sh check --source-only
```

설치는 워크스페이스의 `bag/`, `tracks/`, `results/`를 준비합니다. 소스는 AAC의 실제 분류 경로에서 직접 불러오므로 루트에 `tools/`, `scripts/`, `config/` 별칭을 만들지 않습니다. 녹화 데이터는 기존 AAC 이름인 `bag/` 하나를 사용합니다.

서브모듈은 원본의 읽기 권한이 필요할 수 있습니다. 원본 bag·대형 지도·가상환경·빌드 결과는 GitHub에 포함하지 않습니다. 소형 테스트 입력은 포함합니다. 위 검사는 소스 배치를 확인하며 의존성 설치·실차 연결 완료를 뜻하지 않습니다.

## 기존 본체에서 실행

```bash
cd ~/ros2_ws/src/aac_2026
bash operations/f1tenth/run.sh check
bash operations/f1tenth/run.sh python -m control.acados_mpcc --help
bash operations/f1tenth/run.sh tuner
bash operations/f1tenth/run.sh localization-test
```

기존 AAC PC 환경은 Humble, 가져온 F1TENTH 환경은 Jazzy입니다. 설치 경로를 섞지 않습니다. F1TENTH 빌드는 `ros2_ws/artifacts/f1tenth/<배포판>-<CPU>/`에 구분합니다. AGX 배포·실차 연결은 아직 수행하지 않았습니다.

---

## 기존 AAC 안내

# AI Autonomous Challenge 2026 - JBNU AI Bootcamp

A Docker-based development environment built on Ubuntu 22.04 + ROS2 Humble for building and running autonomous system software.

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Create Workspace & Clone Repository](#2-create-workspace--clone-repository)
3. [Build Docker Image](#3-build-docker-image)
4. [Shell Environment Setup](#4-shell-environment-setup)
5. [Run the Container](#5-run-the-container)
6. [Build Packages](#6-build-packages)
7. [Run AAC Stack](#7-run-aac-stack)
8. [Sensors](#8-sensors)
9. [GUI on Mac (RViz2 / X11)](#9-gui-on-mac-rviz2--x11)
10. [Troubleshooting](#10-troubleshooting)
11. [TODO](#11-todo)
12. [Changelog](#12-changelog)

---

## 1. Prerequisites

### 1-1. Install Docker

```bash
# Remove old versions (if any)
sudo apt-get remove docker docker-engine docker.io containerd runc

# Install using the official script
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Allow running docker without sudo (re-login required)
sudo usermod -aG docker $USER
```

Verify installation:

```bash
docker --version
```

### 1-2. Install NVIDIA Container Toolkit (required for GPU support)

```bash
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list \
  | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

sudo systemctl restart docker
```

Verify installation:

```bash
docker run --rm --gpus all nvidia/cuda:12.8.1-base-ubuntu22.04 nvidia-smi
```

> Reference: [NVIDIA Docker Setup Guide](https://velog.io/@boom109/Nvidia-docker)

---

## 2. Create Workspace & Clone Repository

```bash
mkdir -p /home/${USER}/ros2_ws/src
cd /home/${USER}/ros2_ws/src
git clone --recursive https://github.com/Leedk3/aac_2026.git
cd aac_2026
git checkout main
```

> **Note**: `--recursive` is required to also clone submodules (e.g. `navigation/WildGS-SLAM`).
> Without it, `navigation/WildGS-SLAM/` will be an empty directory.
> If you do not have an SSH key configured, submodule cloning may fail.
> Set up GitHub SSH keys here: [github.com/settings/keys](https://github.com/settings/keys)

### If you forgot `--recursive` (submodule recovery)

If you already cloned without `--recursive` and `navigation/WildGS-SLAM/` is empty, run:

```bash
cd /home/${USER}/ros2_ws/src/aac_2026
git submodule update --init --recursive
```

This fetches and checks out all registered submodules at the pinned commit.

If the submodule directory is missing entirely or the state looks broken:

```bash
cd /home/${USER}/ros2_ws/src/aac_2026

# Step 1: Deinitialize and remove any stale state
git submodule deinit -f navigation/WildGS-SLAM
git rm -rf --cached navigation/WildGS-SLAM 2>/dev/null || true
rm -rf .git/modules/navigation/WildGS-SLAM

# Step 2: Re-initialize and clone
git submodule update --init --recursive
```

---

## 3. Build Docker Image

```bash
cd /home/${USER}/ros2_ws/src/aac_2026/docker
```

### For x86 Desktop (PC / Workstation)

Includes: Ubuntu 22.04, ROS2 Humble, CUDA 12.8 + cuDNN, PyTorch 2.11.0 (cu128), xFormers, Hugging Face Transformers, ONNX Runtime GPU, OpenCV (CUDA), ZeroMQ

```bash
./build_docker.sh x86
```

Resulting image: `aac-2026-image:x86`

> **Note**: The first build may take 30 minutes to over an hour due to OpenCV CUDA compilation.
> Hugging Face models are cached under
> `/ros_ws/src/aac_2026/.hf_home`, so the cache remains available when the
> Docker container is recreated. The directory is excluded from Git.

### For Mac (Apple Silicon — M1/M2/M3/M4/M5)

Includes: Ubuntu 22.04 ARM64, ROS2 Humble, PyTorch 2.1.0 (CPU), OpenCV (apt), ZeroMQ

```bash
./build_docker.sh mac
```

Resulting image: `aac-2026-image:mac`

> **Note**: No CUDA support. GPU-accelerated features (CUDA inference, CUDA OpenCV) will not be available.

### For Jetson (Edge Computing Board)

```bash
./build_docker.sh ros2
```

Resulting image: `aac-2026-image:jetson`

---

## 4. Shell Environment Setup

Add the following to `~/.bashrc` to configure ROS2 environment variables and container launch aliases.

```bash
# ROS2 DDS middleware
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export RMW_CYCLONEDDS_URI=file:///home/${USER}/.cyclonedds.xml" >> ~/.bashrc

# ROS Domain ID (used to isolate namespaces in multi-robot setups)
echo "export ROS_DOMAIN_ID=3" >> ~/.bashrc

# Network interface (replace with your actual interface, e.g., eth0, wlan0)
echo "export DDS_INTERFACE=eth0" >> ~/.bashrc
```

### Container launch alias — x86 Desktop

```bash
echo "alias aac_enter='cd /home/${USER}/ros2_ws/src/aac_2026/docker && bash run_x86.sh'" >> ~/.bashrc
```

### Container launch alias — Mac (Apple Silicon)

```bash
echo "alias aac_enter='cd /home/${USER}/ros2_ws/src/aac_2026/docker && bash run_mac.sh'" >> ~/.bashrc
```

> **GUI on Mac (RViz2)**: See the setup steps below. Mac Docker Desktop runs containers inside a Linux VM, so the standard `/tmp/.X11-unix` socket mount does not work. TCP-based X11 forwarding via XQuartz is required.

### Container launch alias — Jetson

```bash
echo "alias aac_enter='cd /home/${USER}/ros2_ws/src/aac_2026/docker && bash run_jetson.sh'" >> ~/.bashrc
```

### Stop all containers alias (all platforms)

```bash
echo "alias kill_sys='bash /home/${USER}/ros2_ws/src/aac_2026/kill_sys.sh'" >> ~/.bashrc
```

### Stop the tmux/Gazebo session inside the container

```bash
bash /ros_ws/src/aac_2026/stop.sh
```

### Check tmux/Gazebo shutdown harness inside the container

```bash
cd /ros_ws/src/aac_2026
operations/harnesses/tmux_shutdown_harness.sh
operations/harnesses/tmux_shutdown_harness.sh --live
```

Apply changes:

```bash
source ~/.bashrc
```

> **How to check your network interface name**:
>
> ```bash
> ip link show
> ```
>
> Look for the active interface (e.g., `eth0`, `ens3`, `wlan0`) and update `DDS_INTERFACE` accordingly.

---

## 5. Run the Container

```bash
aac_enter
```

The script handles all container states automatically:

| State                           | Behavior                                   |
|---------------------------------|--------------------------------------------|
| Container does not exist        | Creates and starts a new container         |
| Container exists but is stopped | Restarts and attaches                      |
| Container is already running    | Opens a new shell in the running container |

The working directory inside the container is `/ros_ws`, which is mounted from `/home/${USER}/ros2_ws` on the host.

---

## 6. Build Packages

After entering the container, build all packages:

```bash
aac_enter
cd /ros_ws/src/aac_2026
./build.sh
```

`build.sh` first creates `COLCON_IGNORE` files for external ML repositories
such as `navigation/WildGS-SLAM`, `perception/BiSeNet`, `perception/PIDNet`,
`perception/sam3`, and the local `wildgs-env` virtual environment. This keeps
colcon from treating third-party CUDA/Python projects as ROS packages.

To build a specific package only:

```bash
aac_enter
cd /ros_ws
src/aac_2026/perception/setup_colcon_ignores.sh
colcon build --packages-up-to example_cpp_node
```

---

## 7. Run AAC Stack

Inside the container, build once or after code/config changes, then start the
current Gazebo autonomy stack with `start.sh`:

```bash
aac_enter
cd /ros_ws/src/aac_2026
./build.sh
./start.sh
```

If the workspace is already built, you can skip `./build.sh` and run
`./start.sh` directly.

`start.sh` loads `operations/tmuxp_config/config.yaml` and creates the tmux
session `AAC_2026`. The current session starts:

| Pane | Command |
|------|---------|
| Vehicle / Gazebo | `ros2 launch aac_vehicle spawn_vehicle.launch.py` |
| Joystick control | `ros2 launch joy_control_node joy_control.launch.py` |
| IPM BEV | `ros2 launch ipm_bev_node deploy.launch.py` |
| Visual localization (OpenCV) | `ros2 launch image_match_node opencv.launch.py` |

Useful checks:

```bash
tmux attach -t AAC_2026
ros2 topic list | grep ipm_bev
rqt_image_view /ipm_bev/debug
ros2 topic hz /ipm_bev/fused
```

Stop the stack with the shutdown helper instead of killing tmux directly:

```bash
cd /ros_ws/src/aac_2026
./stop.sh
```

`stop.sh` sends `Ctrl-C` to every tmux pane first, then cleans up leftover
Gazebo/ROS processes if they did not exit cleanly.

---

## 8. Sensors

The following sensors are supported out of the box.

### 8-1. USB Camera (×2)

Package: `sensors/usb_cam`

|            | Camera 1               | Camera 2               |
|------------|------------------------|------------------------|
| Device     | `/dev/video0`          | `/dev/video2`          |
| Frame ID   | `camera`               | `camera2`              |
| Resolution | 640×480 @ 30 fps       | 640×480 @ 15 fps       |
| Config     | `config/params_1.yaml` | `config/params_2.yaml` |

**Published topics** (per camera):

```text
/camera1/image_raw
/camera1/image_compressed
/camera1/camera_info
/camera2/image_raw
/camera2/image_compressed
/camera2/camera_info
```

**Launch both cameras:**

```bash
# Inside container
ros2 launch usb_cam camera.launch.py
```

**Verify devices are detected on host before entering container:**

```bash
ls /dev/video*
# Expected: /dev/video0  /dev/video2
```

> **Note**: USB cameras are passed through the `--device /dev/video*` flags in `run_x86.sh`.
> If your camera appears on a different `/dev/videoN`, update `video_device` in the corresponding `config/params_N.yaml`.

---

### 8-2. RPLiDAR (2D LiDAR)

Package: `sensors/rplidar_ros2`

| Item            | Value                                |
|-----------------|--------------------------------------|
| Device          | `/dev/ttyUSB0` (default)             |
| Baudrate        | 256000 (A3)                          |
| Frame ID        | `laser`                              |
| Published topic | `/scan` (`sensor_msgs/LaserScan`)    |

**Launch:**

```bash
# Inside container — RPLiDAR A3
ros2 launch rplidar_ros rplidar_a3.launch.py

# Or generic (A1/A2)
ros2 launch rplidar_ros rplidar.launch.py
```

**Verify device on host:**

```bash
ls /dev/ttyUSB*
# Expected: /dev/ttyUSB0
```

> **Note**: Serial devices are passed via `--device /dev/ttyUSB*` in `run_x86.sh`.

---

## 9. GUI on Mac (RViz2 / X11)

Mac Docker Desktop runs containers inside a Linux VM, so mounting `/tmp/.X11-unix` does not work.
Instead, `run_mac.sh` uses TCP-based X11 via XQuartz (`DISPLAY=host.docker.internal:0`).

### Setup (one-time)

**Step 1** — Install XQuartz:

```bash
brew install --cask xquartz
```

**Step 2** — Enable network connections in XQuartz:

Open XQuartz → **Preferences** → **Security** tab → check **"Allow connections from network clients"**

Then **quit and relaunch XQuartz** for the setting to take effect.

**Step 3** — Allow the container to connect (run this every time after XQuartz starts):

```bash
xhost +localhost
```

**Step 4** — Start the container and launch RViz2:

```bash
aac_enter
rviz2
```

> **Note**: `xhost +localhost` must be re-run each time XQuartz restarts. You can add it to your Mac login items or shell profile to automate this.

---

## 10. Troubleshooting

### `permission denied` when running Docker

```bash
sudo usermod -aG docker $USER
# Log out and log back in
```

### `Cannot connect to the X server` (GUI applications)

```bash
xhost +local:docker
```

### Container name conflict

```bash
docker rm aac-2026-container-all
```

### Check running containers

```bash
docker ps -a
```

### Check available images

```bash
docker images
```

### Camera not detected (`/dev/videoN` missing)

```bash
# Check which video devices are present
ls -l /dev/video*
v4l2-ctl --list-devices

# If the device index differs from params_1.yaml / params_2.yaml, update video_device accordingly
```

### RPLiDAR not detected (`/dev/ttyUSB0` missing)

```bash
# Check USB serial devices
ls -l /dev/ttyUSB*
dmesg | grep tty | tail -10
```

---

## 11. TODO

- [ ] **USB device symlink (udev rules)**
  - Set up persistent symlinks so sensor device paths are stable across reboots and re-plugs.
  - RPLiDAR rule already exists at `sensors/rplidar_ros2/scripts/rplidar.rules` → creates `/dev/rplidar`.
  - Camera symlinks need to be added (e.g., `/dev/camera_front`, `/dev/camera_rear`) based on `idVendor`/`idProduct` or `serial` attributes.
  - After rules are created, update `params_1.yaml` and `params_2.yaml` `video_device` fields to use the symlink paths.

---

## 12. Changelog

### 2026-06-10 — OpenCV visual localization + Chamfer refinement

- `image_match_node opencv.launch.py` added to `operations/tmuxp_config/config.yaml`.
  `./start.sh` now launches the OpenCV SIFT localization node automatically as the fourth pane.
- Added **Chamfer distance refinement** to the OpenCV backend (`opencv_node/ros_wrapper.py`):
  - Map lane mask EDT precomputed once at startup.
  - After RANSAC, a coarse-to-fine coordinate descent (3-DOF: dx, dy, dθ) minimises
    the mean EDT of projected BEV lane pixels — no external dependencies (numpy only).
  - Controlled by `enable_chamfer_refinement`, `chamfer_max_pts`, `chamfer_score_gate`
    in `config/opencv_gazebo.yaml`.
  - New topics: `/visual_localization/opencv/chamfer_score`,
    `/visual_localization/opencv/chamfer_refined_score`.

### 2026-06-08 — Gazebo runtime and perception updates

- x86 Docker base updated to CUDA 12.8.1 with PyTorch 2.11.0 cu128 for RTX 50-series GPUs.
- Added `perception/setup_colcon_ignores.sh` and wired it into `build.sh` to keep third-party ML repositories out of colcon discovery.
- Added `perception/ipm_bev_node` for front/rear camera TF-based IPM BEV generation.
- `operations/tmuxp_config/config.yaml` now starts Gazebo vehicle spawn, joystick control, and `ipm_bev_node`.
- Use `./start.sh` to launch the tmux stack and `./stop.sh` for clean Gazebo/ROS shutdown.

### 2026-06-01 — Migration from `irdl_tiny` scaffold

**Docker scripts** (`docker/`):

- All image names changed: `irdl-tiny-image` → `aac-2026-image`
- All container names changed: `irdl-container-*` → `aac-2026-container-*`
- Variable `irdl_tiny_DIR` renamed to `AAC_DIR` in `run_x86.sh`, `run_mac.sh`, `run_jetson.sh`
- `CYCLONEDDS_URI` path corrected: `/ros_ws/src/irdl_tiny/` → `/ros_ws/src/aac_2026/`
- `env.sh` startup message updated
- Affected files: `build_docker.sh`, `run_x86.sh`, `run_mac.sh`, `run_jetson.sh`, `env.sh`

**Dockerfiles** (`docker/dockerfiles/`):

- `COPY . /ros_ws/src/irdl_tiny` → `COPY . /ros_ws/src/aac_2026` in all three Dockerfiles

**Build fixes** (`sensors/rplidar_ros2/`):

- `CMakeLists.txt`: Removed `-pedantic` flag (caused errors in third-party SDK)
- `sdk/src/arch/linux/net_socket.cpp:170`: `ans<=0` → `ans==nullptr` (pointer vs integer comparison bug)
- `sdk/src/rplidar_driver.cpp:1793, 1822`: `switch (_dataEvt.wait(...))` → `switch ((int)_dataEvt.wait(...))` (narrowing conversion fix)

**Camera launch** (`sensors/usb_cam/launch/`):

- `camera.launch.py`: Added `camera2` node (`params_2.yaml`, `/dev/video2`)
