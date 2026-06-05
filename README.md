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
7. [Sensors](#7-sensors)
8. [GUI on Mac (RViz2 / X11)](#8-gui-on-mac-rviz2--x11)
9. [Troubleshooting](#9-troubleshooting)
10. [TODO](#10-todo)
11. [Changelog](#11-changelog)

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
docker run --rm --gpus all nvidia/cuda:12.1.1-base-ubuntu22.04 nvidia-smi
```

> Reference: [NVIDIA Docker Setup Guide](https://velog.io/@boom109/Nvidia-docker)

---

## 2. Create Workspace & Clone Repository

```bash
mkdir -p /home/${USER}/ros_ws/src
cd /home/${USER}/ros_ws/src
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
cd /home/${USER}/ros_ws/src/aac_2026
git submodule update --init --recursive
```

This fetches and checks out all registered submodules at the pinned commit.

If the submodule directory is missing entirely or the state looks broken:

```bash
cd /home/${USER}/ros_ws/src/aac_2026

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
cd /home/${USER}/ros_ws/src/aac_2026/docker
```

### For x86 Desktop (PC / Workstation)

Includes: Ubuntu 22.04, ROS2 Humble, CUDA 12.1 + cuDNN 8, PyTorch 2.1.0 (CUDA), OpenCV (CUDA), ZeroMQ

```bash
./build_docker.sh x86
```

Resulting image: `aac-2026-image:x86`

> **Note**: The first build may take 30 minutes to over an hour due to OpenCV CUDA compilation.

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
echo "alias aac_enter='cd /home/${USER}/ros_ws/src/aac_2026/docker && bash run_x86.sh'" >> ~/.bashrc
```

### Container launch alias — Mac (Apple Silicon)

```bash
echo "alias aac_enter='cd /home/${USER}/ros_ws/src/aac_2026/docker && bash run_mac.sh'" >> ~/.bashrc
```

> **GUI on Mac (RViz2)**: See the setup steps below. Mac Docker Desktop runs containers inside a Linux VM, so the standard `/tmp/.X11-unix` socket mount does not work. TCP-based X11 forwarding via XQuartz is required.

### Container launch alias — Jetson

```bash
echo "alias aac_enter='cd /home/${USER}/ros_ws/src/aac_2026/docker && bash run_jetson.sh'" >> ~/.bashrc
```

### Stop all containers alias (all platforms)

```bash
echo "alias kill_sys='bash /home/${USER}/ros_ws/src/aac_2026/kill_sys.sh'" >> ~/.bashrc
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

The working directory inside the container is `/ros_ws`, which is mounted from `/home/${USER}/ros_ws` on the host.

---

## 6. Build Packages

After entering the container, build all packages:

```bash
aac_enter
cd /ros_ws/src/aac_2026
./build.sh
```

To build a specific package only:

```bash
aac_enter
cd /ros_ws
colcon build --packages-up-to example_cpp_node
```

---

## 7. Sensors

The following sensors are supported out of the box.

### 7-1. USB Camera (×2)

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

### 7-2. RPLiDAR (2D LiDAR)

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

## 8. GUI on Mac (RViz2 / X11)

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

## 9. Troubleshooting

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

## 10. TODO

- [ ] **USB device symlink (udev rules)**
  - Set up persistent symlinks so sensor device paths are stable across reboots and re-plugs.
  - RPLiDAR rule already exists at `sensors/rplidar_ros2/scripts/rplidar.rules` → creates `/dev/rplidar`.
  - Camera symlinks need to be added (e.g., `/dev/camera_front`, `/dev/camera_rear`) based on `idVendor`/`idProduct` or `serial` attributes.
  - After rules are created, update `params_1.yaml` and `params_2.yaml` `video_device` fields to use the symlink paths.

---

## 11. Changelog

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
