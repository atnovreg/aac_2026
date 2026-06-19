---
marp: true
html: true
theme: default
paginate: true
footer: "방산개방형아키텍처 | Dept. of Advanced Defense Technology and Industry | Prof. Daegyu Lee"
style: |
  section {
    justify-content: flex-start;
    padding: 30px 40px 30px 40px;
  }

  footer {
    position:absolute;
    bottom:20px;
    left:0;
    right:0;
    text-align:center;
    font-size:10pt;
    color:#666;
  }

  h1 {
    font-size:26pt;
    margin-bottom:14px;
  }

  h2 {
    color:#4f81bd;
    font-size:18pt;
  }

  p, li {
    font-size:15pt;
    line-height:1.35;
  }

  pre, code {
    font-size:11pt;
  }

  .cols2 {
    display:grid;
    grid-template-columns:1fr 1fr;
    gap:20px;
  }

  .small p, .small li {
    font-size:13pt;
  }

  .tiny p, .tiny li {
    font-size:11.5pt;
  }

  .flow {
    display:flex;
    align-items:center;
    gap:8px;
    margin:18px 0;
  }

  .box {
    background:#4f81bd;
    color:white;
    padding:12px 16px;
    border-radius:6px;
    font-size:12pt;
    font-weight:bold;
    text-align:center;
    flex:1;
  }

  .box.orange {
    background:#e67e22;
  }

  .box.green {
    background:#27ae60;
  }

  .box.gray {
    background:#555;
  }

  .arrow {
    font-size:22pt;
    color:#aaa;
  }

  table {
    width:100%;
    border-collapse:collapse;
  }

  td, th {
    border:1px solid #cccccc;
    padding:6px;
    font-size:12.5pt;
  }

  th {
    background:#4f81bd;
    color:white;
  }

---

# `Lecture 00: AAC2026 Course Plan`
## 8-10시간 ROS 2 자율주행 프로젝트 수업 설계

방산개방형아키텍처

---

# `1. 이 수업의 한 문장`

이 수업은 AAC2026 코드를 이용해 학생이 자율주행 시스템의 데이터 흐름을
직접 실행하고 수정해보는 수업이다.

```text
환경을 만든다.
노드를 실행한다.
토픽을 확인한다.
차량 모델을 바꾼다.
센서 데이터를 본다.
항법과 제어를 연결한다.
```

---

# `2. 수업 목표`

수업이 끝나면 학생은 다음 질문에 답할 수 있어야 한다.

- Docker는 왜 쓰는가?
- ROS 2 node, topic, launch는 어떻게 연결되는가?
- joystick 입력은 어떻게 차량 명령이 되는가?
- Gazebo 차량 모델의 크기, 센서 위치, 조향 제한은 어디서 바꾸는가?
- camera와 LiDAR topic은 어떻게 확인하는가?
- 영상 기반 위치추정과 LiDAR 기반 위치추정은 무엇이 다른가?
- planner와 tracker는 무엇이 다르고 어떻게 연결되는가?

---

# `3. 전체 흐름`

<div class="flow">
  <div class="box">Ubuntu<br>Docker</div>
  <div class="arrow">→</div>
  <div class="box">ROS 2<br>기초</div>
  <div class="arrow">→</div>
  <div class="box">Joystick<br>Control</div>
  <div class="arrow">→</div>
  <div class="box orange">Gazebo<br>Vehicle</div>
  <div class="arrow">→</div>
  <div class="box green">Navigation<br>Planning</div>
</div>

강의 설계 원칙:

```text
명령어를 먼저 외우지 않는다.
데이터가 어디서 와서 어디로 가는지 먼저 본다.
```

---

# `4. 10시간 권장 편성`

| 시간 | 주제 | 핵심 결과 |
|---:|---|---|
| 0.5h | Course overview | 시스템 데이터 흐름 이해 |
| 1.0h | Ubuntu/NVIDIA/Docker | AAC 컨테이너 실행 |
| 1.0h | ROS 2 개발 생태계 | pub/sub, run, launch |
| 1.0h | Joystick control | `/joy` → `/ackermann_drive` |
| 1.3h | Gazebo vehicle modeling | URDF/Xacro, Ackermann |
| 0.8h | Camera/LiDAR sensors | image/scan topic 확인 |
| 1.2h | Vision navigation | IPM + image matching |
| 1.2h | LiDAR navigation | 2D/3D SLAM 조사와 실습 |
| 1.0h | Planning | global/local planning |
| 1.0h | Path tracking | pure pursuit/MPC 연결 |

---

# `5. 8시간 압축 편성`

| 시간 | 주제 | 압축 방법 |
|---:|---|---|
| 1.5h | 개발환경 + Docker | 기존 `lecture01` 중심 |
| 1.0h | ROS 2 기초 | pub/sub + launch만 |
| 1.0h | Joystick control | 코드 리뷰 + 실습 |
| 1.2h | Gazebo 차량/센서 | 차량 모델과 센서 모델 통합 |
| 1.2h | 영상 항법 | IPM + OpenCV backend 중심 |
| 1.0h | LiDAR 항법 | 2D 실습, 3D는 조사형 |
| 1.1h | Planning/Tracking | Pure Pursuit skeleton 구현 |

---

# `6. 수업 산출물`

학생이 최종적으로 다뤄야 할 산출물:

- Docker 기반 AAC2026 개발환경
- 직접 실행 가능한 ROS 2 workspace
- joystick으로 움직이는 Gazebo 차량
- 수정 가능한 차량 및 센서 설정
- camera/LiDAR topic 확인 로그
- 영상 matching 기반 localization 결과 해석
- LiDAR SLAM 기술 비교표
- local path tracking 노드 초안

---

# `7. 저장소에서 볼 큰 구조`

```text
aac_2026/
├── control/      # joystick, control command
├── sensors/      # usb camera, rplidar
├── perception/   # IPM, segmentation, road mask
├── navigation/   # image matching, SLAM
├── system/       # Gazebo vehicle, static TF, track
├── operations/   # tmuxp, service, PC setup
├── docker/       # x86/mac/jetson image
└── lecture/      # Marp lecture materials
```

---

# `8. 학생에게 먼저 보여줄 그림`

```text
/joy
  ↓
joy_control_node
  ↓
/ackermann_drive
  ↓
vehicle_state_publisher
  ↓
/ackermann_cmd
  ↓
ros_gz_bridge
  ↓
Gazebo AckermannSteering plugin
  ↓
/odom, /camera_front/image_raw, /scan
```

핵심 질문:

```text
내가 누른 조이스틱 버튼이 어떤 토픽 이름으로 바뀌는가?
```

---

# `9. Lecture 01`
## Ubuntu, NVIDIA Driver, Docker, AAC 개발환경

기존 파일:

```text
lecture/lecture01_dev_environment_setup.md
```

수업 목표:

- Ubuntu 22.04 권장 이유
- NVIDIA driver와 CUDA runtime의 역할 구분
- Docker Engine 설치
- NVIDIA Container Toolkit 설치
- AAC Docker image build
- `aac_enter`, `build.sh`, `start.sh`

---

# `10. Lecture 01 코드/명령 포인트`

학생이 실행해야 할 명령:

```bash
# 실행 위치: Host Ubuntu에서 시작, 이후 AAC Container
nvidia-smi
docker --version
docker run --rm hello-world
docker run --rm --gpus all nvidia/cuda:12.8.1-base-ubuntu22.04 nvidia-smi

cd ~/ros_ws/src/aac_2026/docker
./build_docker.sh x86
aac_enter
# aac_enter 이후부터는 AAC Container
cd /ros_ws/src/aac_2026
./build.sh
```

확인 질문:

```text
GPU driver는 host에 있는가, container에 있는가?
```

---

# `11. Docker 생태계 설명`

이 프로젝트의 중요한 관점:

```text
Docker image = ROS 2/CUDA/OpenCV/PyTorch 실행환경
Git src      = 학생이 수정하고 제출하는 코드
```

즉, 컨테이너는 코드를 보관하는 장소가 아니라 코드를 실행하는 실험실이다.

---

# `12. Docker bind mount`

AAC2026의 개발 방식:

```text
Host:      /home/${USER}/ros_ws
Container: /ros_ws
```

학생이 host에서 VS Code로 수정하면 container 안에서도 같은 파일이 보인다.

```text
편집은 host에서 한다.
빌드와 실행은 container에서 한다.
src는 Git으로 관리한다.
```

---

# `13. Lecture 02`
## ROS 2 개발환경과 통신 모델

수업 목표:

- workspace, package, node 개념
- topic pub/sub 개념
- message type 확인
- `ros2 run`과 `ros2 launch`의 차이
- `colcon build`와 `source install/setup.bash`

---

# `14. ROS 2에서 node란?`

node는 한 가지 역할을 하는 실행 프로그램이다.

예시:

| Node | 역할 |
|---|---|
| `joy_node` | joystick device를 읽어 `/joy` 발행 |
| `joy_control_node` | `/joy`를 차량 명령으로 변환 |
| `vehicle_state_publisher` | 차량 명령과 상태 처리 |
| `ipm_bev_node` | camera image를 BEV image로 변환 |
| `opencv_localization` | BEV image와 map image matching |

---

# `15. ROS 2 topic이란?`

topic은 node 사이의 데이터 통로다.

```text
publisher node  ->  topic  ->  subscriber node
```

예시:

```text
joy_node publishes /joy
joy_control_node subscribes /joy
joy_control_node publishes /ackermann_drive
```

---

# `16. Topic 확인 명령`

```bash
# 실행 위치: AAC Container
ros2 topic list
ros2 topic info /ackermann_drive
ros2 interface show ackermann_msgs/msg/AckermannDriveStamped
ros2 topic echo /ackermann_drive --once
ros2 topic hz /ackermann_drive
```

학생이 반드시 익혀야 할 패턴:

```text
이름 확인 -> 타입 확인 -> 메시지 구조 확인 -> 값 확인 -> 주기 확인
```

---

# `17. ros2 run vs ros2 launch`

| 명령 | 쓰는 상황 |
|---|---|
| `ros2 run` | 노드 하나를 직접 실행 |
| `ros2 launch` | 여러 노드와 파라미터를 한 번에 실행 |

예시:

```bash
# 실행 위치: AAC Container
ros2 run joy_control_node joy_control_node_exe
ros2 launch joy_control_node joy_control.launch.py
```

---

# `18. Lecture 03`
## Joystick Control Node

중심 파일:

```text
control/joy_control_node/src/joy_control.cpp
control/joy_control_node/launch/joy_control.launch.py
control/joy_control_node/config/joy_control_params.yaml
```

오늘의 질문:

```text
버튼과 스틱 값은 어떻게 speed, steering_angle이 되는가?
```

---

# `19. Joystick topic 흐름`

```text
PS4/PS3 joystick
  ↓
joy_node
  ↓ /joy (sensor_msgs/Joy)
joy_control_node
  ↓ /ackermann_drive (ackermann_msgs/AckermannDriveStamped)
vehicle_state_publisher
```

실습:

```bash
# 실행 위치: AAC Container
ros2 launch joy_control_node joy_control.launch.py
ros2 topic echo /joy
ros2 topic echo /ackermann_drive
```

---

# `20. joy_control_node 코드: publisher/subscriber`

```cpp
pub_ackermann_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
  "/ackermann_drive", rclcpp::QoS(10));

sub_joy_ = create_subscription<sensor_msgs::msg::Joy>(
  "/joy", rclcpp::QoS(10),
  std::bind(&JoyControl::joyCallback, this, std::placeholders::_1));

sub_auto_cmd_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
  "/ackermann_auto", rclcpp::QoS(10),
  std::bind(&JoyControl::autoCallback, this, std::placeholders::_1));
```

설명 포인트:

- 하나의 노드가 subscriber이면서 publisher일 수 있다.
- `/ackermann_auto`는 미래 planner와 연결될 입구다.

---

# `21. joy_control_node 코드: timer`

```cpp
timer_ = create_wall_timer(
  25ms, std::bind(&JoyControl::timerCallback, this));
```

25 ms마다 실행된다.

```text
1000 ms / 25 ms = 40 Hz
```

수업 질문:

```text
joystick callback에서 바로 publish하지 않고 timer에서 publish하는 이유는?
```

---

# `22. RUN / STOP / E-STOP`

```cpp
if (buttons.at(1) == 1 && !run_mode_) {
  run_mode_ = true;
} else if (buttons.at(0) == 1 && run_mode_) {
  run_mode_ = false;
  ackermann_cmd_.drive.speed = 0.0f;
  ackermann_cmd_.drive.steering_angle = 0.0f;
  pub_ackermann_->publish(ackermann_cmd_);
  return;
}
```

핵심:

- Circle: RUN MODE
- Cross: E-STOP
- E-STOP은 속도와 조향각을 즉시 0으로 publish한다.

---

# `23. Manual vs Auto`

```cpp
if (buttons.at(5) == 1) {
  ackermann_cmd_.drive.speed =
    static_cast<float>(maximum_speed_ * axes.at(4));
  ackermann_cmd_.drive.steering_angle =
    static_cast<float>(reverse * maximum_steering_angle_ * axes.at(0));
} else if (has_auto_cmd_) {
  ackermann_cmd_.drive.speed = auto_cmd_.drive.speed;
  ackermann_cmd_.drive.steering_angle =
    static_cast<float>(reverse) * auto_cmd_.drive.steering_angle;
}
```

설명:

```text
R1 hold = 사람이 직접 조종
R1 release = planner/autonomy command pass-through
```

---

# `24. 수정 실습`

파일:

```text
control/joy_control_node/config/joy_control_params.yaml
```

수정할 값:

```yaml
maximum_speed: 1.5
maximum_steering_angle: 0.524
reverse_angle: false
```

질문:

```text
maximum_speed를 0.5로 줄이면 `/ackermann_drive`는 어떻게 바뀌는가?
```

---

# `25. Lecture 04`
## Gazebo 차량 환경과 차량 모델링

중심 파일:

```text
system/aac_vehicle/urdf/aac_vehicle.urdf.xacro
system/aac_vehicle/urdf/sensors.urdf.xacro
system/aac_vehicle/config/vehicle_params.yaml
system/aac_vehicle/config/sensor_config.yaml
system/aac_vehicle/launch/spawn_vehicle.launch.py
```

오늘의 질문:

```text
시뮬레이터 속 차량은 어디에 정의되어 있는가?
```

---

# `26. Gazebo 실행 흐름`

```text
spawn_vehicle.launch.py
  ├── sensor_config.yaml 읽기
  ├── xacro -> robot_description 생성
  ├── robot_state_publisher 실행
  ├── Gazebo world 실행
  ├── robot_description으로 차량 spawn
  ├── ros_gz_bridge 실행
  └── vehicle_state_publisher 실행
```

실습:

```bash
# 실행 위치: AAC Container
ros2 launch aac_vehicle spawn_vehicle.launch.py
```

---

# `27. URDF/Xacro에서 차량 파라미터`

```xml
<xacro:property name="wheelbase"      value="0.80"/>
<xacro:property name="track_width"    value="0.46"/>
<xacro:property name="wheel_radius"   value="0.08"/>
<xacro:property name="chassis_length" value="0.99"/>
<xacro:property name="chassis_width"  value="0.55"/>
<xacro:property name="max_steering_angle" value="0.524"/>
```

학생이 바꿔볼 값:

- wheelbase
- track width
- wheel radius
- max steering angle

---

# `28. 차량 좌표계`

```text
base_link:
  +X forward
  +Y left
  +Z up

base_footprint:
  ground reference
```

차량 모델링에서 가장 중요한 일:

```text
모든 센서와 바퀴 위치를 같은 좌표계 기준으로 생각한다.
```

---

# `29. Ackermann 차량 모델`

Ackermann 차량은 앞바퀴가 조향하고 뒷바퀴가 구동한다.

```text
조향각 delta가 커지면 회전 반경 R이 작아진다.
R = L / tan(delta)
```

AAC2026에서 주요 파라미터:

| Symbol | Code parameter |
|---|---|
| L | `wheelbase` |
| W | `track_width` |
| r | `wheel_radius` |
| delta max | `max_steering_angle` |

---

# `30. Gazebo AckermannSteering plugin`

```xml
<plugin filename="ignition-gazebo-ackermann-steering-system"
        name="ignition::gazebo::systems::AckermannSteering">
  <left_joint>rear_left_wheel_joint</left_joint>
  <right_joint>rear_right_wheel_joint</right_joint>
  <left_steering_joint>front_left_steering_joint</left_steering_joint>
  <right_steering_joint>front_right_steering_joint</right_steering_joint>
  <wheel_base>${wheelbase}</wheel_base>
  <wheel_separation>${track_width}</wheel_separation>
  <wheel_radius>${wheel_radius}</wheel_radius>
  <topic>/ackermann_cmd</topic>
</plugin>
```

핵심:

```text
Gazebo plugin은 `/ackermann_cmd`를 받는다.
```

---

# `31. ROS command에서 Gazebo command로`

AAC2026에는 한 단계 변환이 있다.

```text
/ackermann_drive
  AckermannDriveStamped
      ↓
vehicle_state_publisher
      ↓
/ackermann_cmd
  geometry_msgs/Twist
      ↓
ros_gz_bridge
      ↓
Gazebo AckermannSteering
```

---

# `32. vehicle_state_publisher 코드`

```cpp
sub_ackermann_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
  "/ackermann_drive", rclcpp::QoS(10),
  std::bind(&VehicleStatePublisher::ackermannCallback, this, _1));

pub_ackermann_cmd_ = create_publisher<geometry_msgs::msg::Twist>(
  "/ackermann_cmd", rclcpp::QoS(10));
```

설명:

- ROS control 쪽은 Ackermann message를 쓴다.
- Gazebo bridge 쪽은 Twist를 사용한다.

---

# `33. 속도/조향 제한`

```cpp
ackermann_cmd_.drive.steering_angle = static_cast<float>(
  std::clamp(static_cast<double>(ackermann_cmd_.drive.steering_angle),
             -max_steering_angle_, max_steering_angle_));

ackermann_cmd_.drive.speed = static_cast<float>(
  std::clamp(static_cast<double>(ackermann_cmd_.drive.speed),
             -max_speed_, max_speed_));
```

수업 질문:

```text
왜 controller와 vehicle 양쪽에서 제한값을 확인해야 하는가?
```

---

# `34. 센서 위치 설정`

파일:

```text
system/aac_vehicle/config/sensor_config.yaml
```

예시:

```yaml
camera_front:
  xyz: [0.45, 0.0, 0.70]
  rpy: [0.0, 0.2617, 0.0]

lidar:
  xyz: [0.0, 0.0, 0.33]
  rpy: [0.0, 0.0, 0.0]
```

핵심:

```text
센서 성능만큼이나 센서 pose가 중요하다.
```

---

# `35. Lecture 05`
## Gazebo Camera / LiDAR Sensor Modeling

중심 파일:

```text
system/aac_vehicle/urdf/sensors.urdf.xacro
system/aac_vehicle/launch/spawn_vehicle.launch.py
perception/ipm_bev_node/src/ipm_bev_node.cpp
```

오늘의 질문:

```text
시뮬레이터의 센서 데이터는 어떤 ROS topic으로 들어오는가?
```

---

# `36. Camera sensor`

```xml
<sensor name="camera_front" type="camera">
  <topic>/camera_front</topic>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.2217</horizontal_fov>
    <image>
      <width>1280</width>
      <height>720</height>
      <format>R8G8B8</format>
    </image>
  </camera>
</sensor>
```

확인:

```bash
# 실행 위치: AAC Container
ros2 topic hz /camera_front/image_raw
rqt_image_view /camera_front/image_raw
```

---

# `37. LiDAR sensor`

```xml
<sensor name="lidar" type="gpu_lidar">
  <topic>/lidar/scan</topic>
  <update_rate>5.5</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <min_angle>-3.14159265</min_angle>
        <max_angle>3.14159265</max_angle>
      </horizontal>
    </scan>
  </lidar>
</sensor>
```

확인:

```bash
# 실행 위치: AAC Container
ros2 topic echo /scan --once
rviz2
```

---

# `38. ros_gz_bridge`

Gazebo message와 ROS message는 타입이 다르다.

```python
'/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'
'/camera_front@sensor_msgs/msg/Image[ignition.msgs.Image'
'/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
```

핵심:

```text
시뮬레이터와 ROS 2 사이에는 bridge가 필요하다.
```

---

# `39. IPM BEV`

IPM은 camera image를 지면 평면 위의 bird's-eye-view로 바꾸는 과정이다.

```text
camera image pixel
  ↓ camera intrinsics
camera ray
  ↓ TF camera -> base_footprint
ground plane intersection
  ↓
BEV image pixel
```

확인:

```bash
# 실행 위치: AAC Container
ros2 launch ipm_bev_node deploy.launch.py
rqt_image_view /ipm_bev/debug
```

---

# `40. IPM 코드 구조`

```cpp
front_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
  front_image_topic, sensor_qos,
  std::bind(&IpmBevNode::frontImageCallback, this, _1));

fused_bev_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
  this->get_parameter("fused_bev_topic").as_string(), sensor_qos);
```

수업 질문:

```text
front/rear camera를 합친 `/ipm_bev/fused`는 왜 localization 입력으로 좋은가?
```

---

# `41. Lecture 06`
## Image Matching 기반 영상 항법

중심 파일:

```text
navigation/image_match_node/opencv_node/ros_wrapper.py
navigation/image_match_node/opencv_node/lane_mask.py
navigation/image_match_node/opencv_node/map_geometry.py
navigation/image_match_node/config/opencv_gazebo.yaml
```

오늘의 질문:

```text
BEV 영상 한 장으로 지도 위 내 위치를 추정할 수 있는가?
```

---

# `42. 영상 항법 입력과 출력`

입력:

```text
/ipm_bev/fused
track.png map image
```

출력:

```text
/visual_localization/opencv/pose
/visual_localization/opencv/odometry
/visual_localization/opencv/debug_matches
/visual_localization/opencv/match_count
```

---

# `43. OpenCV backend pipeline`

```text
map image load
  ↓
SIFT/ORB/AKAZE feature extraction
  ↓
BEV image feature extraction
  ↓
KNN matching + ratio test
  ↓
RANSAC homography
  ↓
pixel_to_world
  ↓
pose publish
```

---

# `44. Feature detector 선택`

```python
if self.feature_type == 'sift':
    return cv2.SIFT_create(nfeatures=self.max_features), cv2.NORM_L2
if self.feature_type == 'orb':
    return cv2.ORB_create(nfeatures=self.max_features), cv2.NORM_HAMMING
return cv2.AKAZE_create(), cv2.NORM_HAMMING
```

수업 질문:

```text
SIFT는 왜 안정적이고, ORB는 왜 빠른가?
```

---

# `45. Lane-aware matching`

```python
mask = cv2.inRange(
    hsv,
    np.asarray(hsv_lower, dtype=np.uint8),
    np.asarray(hsv_upper, dtype=np.uint8),
)
```

차선 영역을 먼저 찾고, feature가 lane 위에 있는지 분류한다.

핵심:

```text
같은 차선끼리, 같은 비차선끼리 matching하면 잘못된 match를 줄일 수 있다.
```

---

# `46. Homography`

homography는 BEV image pixel을 map image pixel로 보내는 2D 변환이다.

```python
homography, inlier_mask = cv2.findHomography(
    bev_points,
    map_points,
    method,
    self.ransac_reproj_threshold,
)
```

질문:

```text
왜 모든 match를 믿지 않고 RANSAC으로 inlier를 고르는가?
```

---

# `47. pixel_to_world`

```python
x = x_bounds[0] + x_norm * (x_bounds[1] - x_bounds[0])
y = y_bounds[0] + y_norm * (y_bounds[1] - y_bounds[0])
```

map image pixel 좌표를 실제 world 좌표로 변환한다.

AAC track:

```text
x range: -6.0 to 6.0
y range: -8.0 to 8.0
```

---

# `48. AI 기반 matching`

OpenCV matching은 keypoint를 먼저 찾고 descriptor를 비교한다.

LoFTR 계열은 detector-free matching을 사용한다.

```text
OpenCV:  detect keypoints -> describe -> match
LoFTR:   image pair -> transformer attention -> semi-dense matches
```

장점:

- 반복 texture나 low texture에서 더 많은 match를 만들 수 있다.
- GPU와 모델 가중치가 필요하다.

---

# `49. 영상 항법 실습`

```bash
# 실행 위치: AAC Container
cd /ros_ws/src/aac_2026
./build.sh
./start.sh

rqt_image_view /ipm_bev/debug
rqt_image_view /visual_localization/opencv/debug_matches
ros2 topic echo /visual_localization/opencv/pose
ros2 topic echo /visual_localization/opencv/match_count
```

확인:

```text
match_count가 높아도 pose가 맞는지 `/odom_gt`와 비교해야 한다.
```

---

# `50. Lecture 07`
## LiDAR 기반 항법: 2D와 3D

오늘의 질문:

```text
LiDAR 항법은 camera 항법과 무엇이 다른가?
```

구분:

| 종류 | 대표 입력 | 대표 출력 |
|---|---|---|
| 2D LiDAR SLAM | `LaserScan` | 2D occupancy map, pose |
| 3D LiDAR odometry/SLAM | `PointCloud2`, IMU | 6-DoF odometry, point cloud map |

---

# `51. 2D LiDAR: SLAM Toolbox`

SLAM Toolbox는 ROS 2에서 많이 쓰는 2D SLAM 패키지다.

필요한 것:

```text
/scan
odom -> base frame TF
map frame
```

출력:

```text
/map
map -> odom 또는 pose estimate
```

AAC2026 실습:

```bash
# 실행 위치: AAC Container
ros2 launch aac_vehicle slam_mapping.launch.py use_sim_time:=true
```

---

# `52. 2D LiDAR 실습 확인`

```bash
# 실행 위치: AAC Container
ros2 topic hz /scan
ros2 topic echo /scan --once
ros2 topic list | grep map
rviz2
```

RViz에서 확인할 display:

- TF
- LaserScan `/scan`
- Map `/map`
- Odometry `/odom`

---

# `53. 3D LiDAR: Cartographer`

Cartographer는 2D와 3D SLAM을 모두 지원하는 SLAM system이다.

수업에서의 위치:

```text
2D LiDAR SLAM의 다음 단계
3D point cloud와 IMU를 사용하는 SLAM 개념 소개
```

설명 포인트:

- local trajectory builder
- submap
- scan matching
- pose graph optimization

---

# `54. 3D LiDAR-Inertial: LIO-SAM`

LIO-SAM은 LiDAR와 IMU를 factor graph로 결합하는 3D 항법 방법이다.

핵심 개념:

```text
LiDAR odometry
IMU preintegration
factor graph
loop closure
```

AAC2026에서 바로 실습하려면 필요한 것:

- 3D LiDAR `PointCloud2`
- IMU topic
- sensor calibration
- 시간 동기화

---

# `55. 3D LiDAR-Inertial: FAST-LIO2`

FAST-LIO2는 raw point를 map에 직접 정합하고, iterated Kalman filter와
incremental k-d tree를 사용하는 빠른 LiDAR-inertial odometry 계열이다.

수업에서의 위치:

```text
최신 3D LiDAR 항법은 대부분 LiDAR 단독보다 IMU 융합을 중요하게 본다.
```

학생 선택 실습:

```text
SLAM Toolbox, Cartographer, LIO-SAM, FAST-LIO2를 입력/출력/필요 센서로 비교하라.
```

---

# `56. LiDAR 항법 비교표`

| 방법 | 차원 | 입력 | 장점 | 수업 적용 |
|---|---:|---|---|---|
| SLAM Toolbox | 2D | LaserScan + odom | ROS 2 실습 쉬움 | 필수 |
| Cartographer | 2D/3D | LaserScan/PointCloud + IMU | 구조 설명 좋음 | 조사/확장 |
| LIO-SAM | 3D | PointCloud2 + IMU | factor graph 기반 | 조사 |
| FAST-LIO2 | 3D | PointCloud2 + IMU | 빠른 odometry | 조사 |

---

# `57. Lecture 08`
## Path Planning과 Local Planning

오늘의 질문:

```text
위치를 알면 차량은 어떻게 갈 길을 정하는가?
```

구분:

```text
global planning: 시작점에서 목표점까지 큰 경로
local planning: 지금 주변 장애물을 고려한 짧은 궤적
path tracking: 이미 주어진 경로를 차량 명령으로 따라감
```

---

# `58. Nav2 구조`

Nav2는 ROS 2 navigation framework다.

```text
map / costmap
  ↓
planner server
  ↓
global path
  ↓
controller server
  ↓
cmd_vel or vehicle command
```

AAC2026에서는 최종적으로:

```text
path -> local tracker -> /ackermann_auto -> joy_control_node -> /ackermann_drive
```

---

# `59. Global planner`

대표 알고리즘:

- Dijkstra
- A*
- Hybrid-A*
- State lattice

Ackermann 차량에서는 단순 grid path보다 차량의 최소 회전 반경이 중요하다.

질문:

```text
차량이 실제로 돌 수 없는 경로를 planner가 만들면 어떤 문제가 생기는가?
```

---

# `60. Nav2 Smac Planner`

Nav2 Smac Planner는 다음 계열을 포함한다.

```text
2D A*
Hybrid-A*
State Lattice
```

수업 적용:

- 2D A*: 개념 설명이 쉽다.
- Hybrid-A*: 차량형 로봇의 heading과 회전 반경을 설명하기 좋다.
- State Lattice: motion primitive 개념 설명에 좋다.

---

# `61. Local planner / controller`

local planning은 현재 주변 장애물과 차량 동역학을 고려해 가까운 미래 명령을 고른다.

대표 접근:

- DWB
- TEB
- Regulated Pure Pursuit
- MPPI
- MPC

강의에서는 Pure Pursuit를 먼저 구현하고, MPPI/MPC는 확장 개념으로 둔다.

---

# `62. MPPI 개념`

MPPI는 여러 후보 trajectory를 sampling하고 비용이 낮은 방향으로 명령을 고른다.

```text
sample controls
  ↓
rollout trajectories
  ↓
score by critics
  ↓
choose command
```

학생에게 강조:

```text
local planner는 "짧은 미래를 많이 상상해보고" 좋은 명령을 고르는 문제다.
```

---

# `63. Lecture 09`
## Local Path Tracking

오늘의 질문:

```text
경로가 주어졌을 때 Ackermann 차량은 어떤 조향각으로 따라가는가?
```

먼저 구현할 알고리즘:

```text
Pure Pursuit
```

출력:

```text
ackermann_msgs/msg/AckermannDriveStamped
```

---

# `64. Pure Pursuit`

Pure Pursuit의 아이디어:

```text
경로 위에서 lookahead distance만큼 앞의 점을 고른다.
차량이 그 점을 향해 원호를 그리며 간다고 가정한다.
필요한 curvature를 계산한다.
curvature를 steering angle로 바꾼다.
```

수식:

```text
curvature = 2 * y_L / Ld^2
steering = atan(wheelbase * curvature)
```

---

# `65. Tracking node skeleton`

새 패키지 후보:

```text
control/path_tracking_node
```

입력:

```text
/planned_path    nav_msgs/Path
/odom            nav_msgs/Odometry
```

출력:

```text
/ackermann_auto  ackermann_msgs/AckermannDriveStamped
```

`joy_control_node`는 R1이 눌리지 않았을 때 `/ackermann_auto`를 통과시킬 수 있다.

---

# `66. Tracking pseudocode`

```python
path = latest_path
pose = latest_odom.pose

target = find_first_point_farther_than(path, pose, lookahead_distance)
target_base = transform_map_to_base(target)

curvature = 2.0 * target_base.y / (lookahead_distance ** 2)
steering = atan(wheelbase * curvature)
speed = target_speed

publish AckermannDriveStamped(speed, steering)
```

---

# `67. Integration test`

```text
planner/path source
  ↓ /planned_path
path_tracking_node
  ↓ /ackermann_auto
joy_control_node
  ↓ /ackermann_drive
vehicle_state_publisher
  ↓ /ackermann_cmd
Gazebo
```

확인:

```bash
# 실행 위치: AAC Container
ros2 topic echo /ackermann_auto
ros2 topic echo /ackermann_drive
ros2 topic echo /odom
```

---

# `68. tmux 기반 전체 실행`

파일:

```text
operations/tmuxp_config/config.yaml
```

현재 실행되는 주요 pane:

```text
static_tf_tree
aac_vehicle spawn_vehicle
joy_control_node
ipm_bev_node
image_match_node opencv
slam_mapping
```

실습:

```bash
# 실행 위치: AAC Container
cd /ros_ws/src/aac_2026
./start.sh
tmux attach -t AAC_2026
```

---

# `69. 학생이 자주 만날 문제`

| 증상 | 확인 |
|---|---|
| Docker 권한 오류 | `groups`, docker group 재로그인 |
| GPU가 container에서 안 보임 | `docker run --gpus all ... nvidia-smi` |
| joystick topic 없음 | `/dev/input/js0`, input group |
| image topic 없음 | Gazebo sensor, bridge, remapping |
| TF 오류 | `ros2 run tf2_tools view_frames` |
| localization pose 이상 | map bounds, flip/swap, inlier ratio |
| 차량이 과하게 돈다 | steering limit, wheelbase, lookahead |

---

# `70. 실습 체크포인트 제안`

체크포인트 1:

```text
maximum_speed와 maximum_steering_angle을 바꾸고 `/ackermann_drive` 변화를 기록하라.
```

체크포인트 2:

```text
camera_front의 pitch를 바꾸고 `/ipm_bev/debug` 변화를 비교하라.
```

체크포인트 3:

```text
OpenCV feature_type을 sift/orb/akaze로 바꾸고 match_count와 pose 안정성을 비교하라.
```

체크포인트 4:

```text
SLAM Toolbox, Cartographer, LIO-SAM, FAST-LIO2를 표로 비교하라.
```

이 수업은 시험이나 점수 기반 평가를 하지 않는다.

```text
목표는 채점이 아니라, 각자 실행하고 관찰하고 수정해보는 경험이다.
```

---

# `71. 영상 삽입 계획`

```text
[VIDEO SLOT 01]
Ubuntu/Docker/GPU check
추천 길이: 60초

[VIDEO SLOT 02]
Joystick으로 Gazebo 차량 조종
추천 길이: 60초

[VIDEO SLOT 03]
sensor_config.yaml 수정 후 camera view 변화
추천 길이: 60초

[VIDEO SLOT 04]
IPM BEV와 OpenCV debug_matches
추천 길이: 90초

[VIDEO SLOT 05]
RViz에서 LaserScan/Map/TF 확인
추천 길이: 90초
```

---

# `72. PPTX export`

Marp Markdown을 pptx로 변환:

```bash
# 실행 위치: Host Ubuntu
cd /home/leedk/ros_ws/src/aac_2026
marp lecture/lecture00_aac2026_course_plan.md \
  --pptx \
  --allow-local-files \
  -o lecture/lecture00_aac2026_course_plan.pptx
```

Marp CLI가 없다면:

```bash
# 실행 위치: Host Ubuntu
npm install -g @marp-team/marp-cli
```

---

# `73. 다음 작업`

우선순위:

1. `lecture01_dev_environment_setup.md` 보강
2. `lecture02_ros2_development_ecosystem.md` 작성
3. `lecture03_joystick_control_node.md` 작성
4. `lecture04_gazebo_vehicle_modeling.md` 작성
5. `lecture05_gazebo_sensor_modeling.md` 작성
6. `lecture06_vision_navigation_image_matching.md` 작성
7. `lecture07_lidar_navigation_2d_3d.md` 작성
8. `lecture08_path_planning_local_planning.md` 작성
9. `lecture09_path_tracking_ackermann.md` 작성

---

# `74. References`

공식 문서와 대표 자료:

- ROS 2 Humble Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- Docker Engine on Ubuntu: https://docs.docker.com/engine/install/ubuntu/
- Docker bind mounts: https://docs.docker.com/engine/storage/bind-mounts/
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
- Ubuntu NVIDIA drivers: https://documentation.ubuntu.com/server/how-to/graphics/install-nvidia-drivers/
- Gazebo sensors: https://gazebosim.org/docs/latest/sensors/
- Nav2: https://docs.nav2.org/
- SLAM Toolbox: https://docs.ros.org/en/humble/p/slam_toolbox/
- Cartographer ROS: https://google-cartographer-ros.readthedocs.io/
- LIO-SAM: https://github.com/TixiaoShan/LIO-SAM
- FAST-LIO2: https://arxiv.org/abs/2107.06829
- OpenCV homography tutorial: https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_feature_homography/py_feature_homography.html
- LoFTR: https://arxiv.org/abs/2104.00680
- EfficientLoFTR: https://huggingface.co/docs/transformers/en/model_doc/efficientloftr
