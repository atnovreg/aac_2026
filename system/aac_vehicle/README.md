# aac_vehicle

ROS2 패키지 — AAC 2026 스케일 카 플랫폼의 Gazebo 시뮬레이션 모델 및 Ackermann 제어 노드.

---

## 차량 제원

| 항목 | 값 |
|------|----|
| 전체 길이 | 99 cm |
| 전체 폭 | 55 cm |
| 전체 높이 | 41 cm |
| 휠베이스 | 80 cm |
| 트랙 폭 (좌우 휠 중심 간격) | 46 cm |
| 휠 반지름 | 8 cm |
| 최대 조향각 | 0.524 rad (≈ 30°) |
| 조향 방식 | Ackermann |

---

## 패키지 구조

```
aac_vehicle/
├── urdf/
│   └── aac_vehicle.urdf.xacro   # 차량 모델 (링크, 조인트, Gazebo 플러그인)
├── config/
│   ├── vehicle_params.yaml      # 차량 파라미터 (휠베이스, 조향각 등)
│   └── slam_mapper_params.yaml  # SLAM 전용 TF 및 mapper 설정
├── launch/
│   ├── spawn_vehicle.launch.py  # Gazebo 실행 + 차량 스폰 + 브릿지
│   └── slam_mapping.launch.py   # Gazebo/실차 공용 SLAM bringup
├── include/aac_vehicle/
│   └── vehicle_state_publisher.h
└── src/
    ├── vehicle_state_publisher.cpp
    └── vehicle_state_publisher_node.cpp
```

---

## URDF 모델

### 링크 구조

```
base_footprint (지면 기준)
└── base_link (차체 CG, 휠 중심 높이)
    ├── front_left_steering_link  ← revolute (Z축, 조향)
    │   └── front_left_wheel      ← continuous (Y축, 회전)
    ├── front_right_steering_link ← revolute (Z축, 조향)
    │   └── front_right_wheel     ← continuous (Y축, 회전)
    ├── rear_left_wheel           ← continuous (Y축, 구동)
    └── rear_right_wheel          ← continuous (Y축, 구동)
```

### 비주얼 구성

| 파트 | 색상 | 설명 |
|------|------|------|
| 하부 플레이트 | 진회색 | 차체 전체 footprint |
| 후방 차체 | 파란색 | 메인 바디 |
| 전방 후드 | 주황색 | 전방 방향 식별 |
| 캐빈 | 반투명 회색 | 상부 캐빈 |
| 조향 링크 지시자 | **RGB 좌표축** | 조향각 시각화 (R=전방, G=좌, B=상) |

> 앞바퀴 조향 링크에 부착된 RGB 좌표축이 조향 관절과 함께 회전하므로, 현재 조향각을 시각적으로 즉시 확인할 수 있다.

---

## Gazebo 플러그인 (Ignition Fortress)

DiffDrive 플러그인 대신 **JointController 방식**을 사용하여 정확한 Ackermann 기구학을 구현한다.

| 플러그인 | 대상 조인트 | 토픽 | 단위 |
|---------|------------|------|------|
| `JointPositionController` | `front_left_steering_joint` | `/joint/front_left_steer` | rad |
| `JointPositionController` | `front_right_steering_joint` | `/joint/front_right_steer` | rad |
| `JointVelocityController` | `rear_left_wheel_joint` | `/joint/rear_left_wheel` | rad/s |
| `JointVelocityController` | `rear_right_wheel_joint` | `/joint/rear_right_wheel` | rad/s |
| `AckermannSteering` odometry | — | `/model/aac_vehicle/odometry` | wheel odometry |
| Gazebo dynamic pose | — | `/world/simple_track_world/dynamic_pose/info` | world ground truth |
| `JointStatePublisher` | — | (joint states) | — |

### DiffDrive vs JointController 비교

| | DiffDrive | JointController (현재) |
|---|---|---|
| 전륜 조향 | 두 바퀴 동일 각도 | 내측/외측 다른 각도 |
| 후륜 구동 | 두 바퀴 동일 속도 | 내측/외측 다른 속도 |
| 정확도 | Ackermann 근사 | 정확한 Ackermann 기구학 |

---

## Ackermann 기구학

`/ackermann_drive`로 받은 평균 조향각 δ를 좌/우 바퀴별 정확한 값으로 변환한다.

```
R = L / tan(δ)          # 선회 반경 (후축 중심 기준)

steer_left  = atan(L / (R - W/2))   # 내측 (큰 각도)
steer_right = atan(L / (R + W/2))   # 외측 (작은 각도)

vel_left  = v * (R - W/2) / (R * r)  # [rad/s]
vel_right = v * (R + W/2) / (R * r)

L: 휠베이스, W: 트랙 폭, r: 휠 반지름
δ > 0 = 좌회전 → 좌측이 내측
```

> δ ≈ 0 (직진) 시: 양쪽 조향각 = 0, 양쪽 휠 속도 = v / r

---

## 토픽

### Subscribe

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/ackermann_drive` | `ackermann_msgs/AckermannDriveStamped` | 속도 + 조향각 명령 |
| `/odom` | `nav_msgs/Odometry` | Ackermann 플러그인의 적분 wheel odometry |

### Publish

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/joint/front_left_steer` | `std_msgs/Float64` | 좌전륜 조향각 [rad] |
| `/joint/front_right_steer` | `std_msgs/Float64` | 우전륜 조향각 [rad] |
| `/joint/rear_left_wheel` | `std_msgs/Float64` | 좌후륜 각속도 [rad/s] |
| `/joint/rear_right_wheel` | `std_msgs/Float64` | 우후륜 각속도 [rad/s] |
| `/vehicle/marker` | `visualization_msgs/Marker` | RViz 차체 마커 |
| `/odom_gt` | `nav_msgs/Odometry` | Gazebo 월드 pose 기반 ground truth (`map` frame) |

---

## 파라미터 (`vehicle_params.yaml`)

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `wheelbase` | 0.80 m | 전후축 거리 |
| `track_width` | 0.46 m | 좌우 휠 중심 간격 |
| `wheel_radius` | 0.08 m | 휠 반지름 |
| `max_steering_angle` | 0.524 rad (30°) | 최대 조향각 (URDF joint limit과 동기화) |
| `max_speed` | 3.0 m/s | 속도 클램프 상한 |

---

## 실행

```bash
# Ignition Gazebo + 차량 스폰 + 브릿지 + vehicle_state_publisher
ros2 launch aac_vehicle spawn_vehicle.launch.py

# map -> odom 및 센서 고정 TF
ros2 launch static_tf_tree static_tf_tree.launch.py use_sim_time:=true

# Gazebo 센서로 SLAM Toolbox 실행
ros2 launch aac_vehicle slam_mapping.launch.py use_sim_time:=true

# Visual localization 추정값으로 map -> odom 생성
ros2 launch image_match_node opencv.launch.py use_sim_time:=true

# 조향 입력 테스트 (수동 publish)
ros2 topic pub /ackermann_drive ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 1.0, steering_angle: 0.3}}"
```

### SLAM 전용 TF

SLAM은 차량의 기본 TF와 충돌하지 않도록 별도 트리를 사용한다.

```text
map -> odom -> base_footprint
map -> odom_lidar -> base_footprint_lidar -> lidar_link_lidar
```

첫 번째 branch의 `map -> odom`은 현재 실험에서 `static_tf_tree`가 identity
고정 TF로 발행한다. `odom -> base_footprint`은 로컬 odometry를 사용한다.
두 번째 branch는 LiDAR SLAM 결과를 비교하거나 보조 odometry로 사용할 수
있도록 분리되어 있다.

원본 `/scan`은 `/scan_lidar`로 복제되며 `frame_id`도 `lidar_link_lidar`로
변경된다. 따라서 Gazebo 또는 실차가 사용하는 `base_footprint`, `base_link`,
LiDAR frame을 SLAM Toolbox가 다시 발행하지 않는다.

실차에서는 Gazebo를 실행하지 않고 실제 odometry와 LaserScan 토픽을 지정한다.
LiDAR 장착 위치는 `static_tf_tree/config/static_tf_tree.yaml`의
`lidar_slam_mount`에서 관리한다.
별도 LiDAR odometry 노드가 `/lidar/odom`을 발행한다면
`input_odom_topic:=/lidar/odom`으로 지정하면 된다.

```bash
ros2 launch aac_vehicle slam_mapping.launch.py \
  use_sim_time:=false \
  input_odom_topic:=/odom \
  input_scan_topic:=/scan
```

실제 localization이 동적인 `map -> odom`을 발행하면 static 설정의
`map_to_odom.enabled`를 `false`로 바꿔야 한다.

---

## 전체 시스템 흐름

```
[조이스틱 / 키보드]
        ↓
  joy_control_node
        ↓
  /ackermann_drive
        ↓
  vehicle_state_publisher
    computeJointCommands()
        ↓
  /joint/front_left_steer   → ros_gz_bridge → JointPositionController (Ignition)
  /joint/front_right_steer  → ros_gz_bridge → JointPositionController (Ignition)
  /joint/rear_left_wheel    → ros_gz_bridge → JointVelocityController (Ignition)
  /joint/rear_right_wheel   → ros_gz_bridge → JointVelocityController (Ignition)

  /model/aac_vehicle/odometry ← ros_gz_bridge ← AckermannSteering (Ignition)
        ↓ (remapped to /odom)
  vehicle_state_publisher (상태 로깅)

  /world/simple_track_world/dynamic_pose/info
        ↓ ros_gz_bridge (/gazebo/dynamic_pose)
  gazebo_ground_truth_publisher
        ↓
  /odom_gt (map → base_link_gt)
```
