# CLAUDE.md — ros2_ws

Behavioral guidelines and project context for the `aac_2026` ROS2 autonomous driving workspace.

---

## 1. Behavioral Guidelines

**Think Before Coding**: State assumptions explicitly. If unclear, ask. Present tradeoffs before implementing.

**Simplicity First**: Minimum code that solves the problem. No speculative features, no premature abstractions.

**Surgical Changes**: Touch only what was asked. Match existing code style. Don't refactor adjacent code.

**Goal-Driven Execution**: Define verifiable success criteria before starting multi-step tasks.

---

## 2. Project Overview — aac_2026

AI Autonomous Challenge 2026 (JBNU AI Bootcamp).
ROS2 Humble + Docker on Ubuntu 22.04. Target platforms: x86 (CUDA), Mac (ARM64, CPU-only), Jetson.

### Directory structure

```
ros2_ws/
└── src/
    └── aac_2026/
        ├── docker/
        │   ├── build_docker.sh       # Build image: ./build_docker.sh [x86|mac|ros2]
        │   ├── run_x86.sh            # Enter container on x86 (NVIDIA GPU)
        │   ├── run_mac.sh            # Enter container on Mac (no GPU)
        │   ├── run_jetson.sh         # Enter container on Jetson
        │   ├── env.sh                # Container entrypoint (sources ROS2 setup)
        │   └── dockerfiles/
        │       ├── x86.Dockerfile    # CUDA 12.1 + cuDNN 8 + OpenCV CUDA + PyTorch 2.1
        │       ├── mac.Dockerfile    # ARM64 CPU-only + OpenCV apt + PyTorch 2.1 CPU
        │       └── jetson.Dockerfile # Jetson-specific image
        ├── example_cpp_node/         # Reference ROS2 C++ node (timer + pub/sub pattern)
        ├── requirements.txt          # Python deps (numpy, torch, open3d, etc.)
        ├── build.sh                  # colcon build wrapper (Debug mode)
        ├── .cyclonedds.xml           # CycloneDDS config
        └── README.md
```

### Docker image names (after aac_2026 migration)

| Platform | Image tag               | Container name           |
|----------|-------------------------|--------------------------|
| x86      | `aac-2026-image:x86`    | `aac-2026-container-all` |
| Mac      | `aac-2026-image:mac`    | `aac-2026-container-mac` |
| Jetson   | `aac-2026-image:jetson` | `aac-2026-container-all` |

---

## 3. Docker Workflow

```bash
# 1. Build image (from host, one-time — takes 30-60 min for x86 due to OpenCV CUDA)
cd ~/ros2_ws/src/aac_2026/docker
./build_docker.sh x86      # or mac / ros2

# 2. Enter container (handles create / restart / attach automatically)
./run_x86.sh               # or run_mac.sh / run_jetson.sh

# 3. Inside container: build ROS2 packages
cd /ros_ws/src/aac_2026
./build.sh

# 4. Source workspace (already in .bashrc inside container)
source /ros_ws/install/setup.bash
```

### Shell alias (add to ~/.bashrc on host)

```bash
# x86
echo "alias aac_enter='cd ~/ros2_ws/src/aac_2026/docker && bash run_x86.sh'" >> ~/.bashrc

# Required env vars
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export RMW_CYCLONEDDS_URI=file:///home/${USER}/.cyclonedds.xml" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=3" >> ~/.bashrc
echo "export DDS_INTERFACE=eth0" >> ~/.bashrc   # adjust interface name
source ~/.bashrc
```

---

## 4. Adding a New ROS2 Package

Use `example_cpp_node` as a template:

```bash
# Inside container
cp -r /ros_ws/src/aac_2026/example_cpp_node /ros_ws/src/aac_2026/my_package
# Edit package.xml, CMakeLists.txt, src/ files
cd /ros_ws && colcon build --packages-select my_package
```

Key patterns from the example node:

- Header in `include/<package_name>/`
- `ament_auto_add_library` + `ament_auto_add_executable`
- `ament_auto_find_build_dependencies()` replaces manual `find_package` for ROS deps
- Launch file in `launch/deploy.launch.py`

---

## 5. Key ROS2 Topics & Conventions

DDS: CycloneDDS (`rmw_cyclonedds_cpp`). Domain ID: 3.

Common topic namespaces seen in the codebase:

- `/Odometry/*` — vehicle/robot odometry
- `/gnss1/`, `/gnss2/` — GNSS fix and odometry
- `/imu/data` — IMU
- `/left/velodyne_points`, `/right/velodyne_points`, `/ouster/points` — LiDAR
- `/Ackermann/veh_state` — vehicle state

---

## 6. Python Dependencies (requirements.txt)

Core: `numpy`, `scipy`, `Pillow`, `pyyaml`, `matplotlib`, `scikit-learn`
Vision: `opencv-python`, `open3d`
Localization: `filterpy`, `utm`
ML: PyTorch installed separately in Dockerfile (CUDA 12.1 for x86)
