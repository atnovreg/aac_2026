---
marp: true
html: true
theme: default
paginate: true
footer: "방산개방형아키텍처 | Dept. of Advanced Defense Technology and Industry | Prof. Daegyu Lee"
style: |
  section { justify-content: flex-start; padding: 30px 40px; }
  footer { position:absolute; bottom:20px; left:0; right:0; text-align:center; font-size:10pt; color:#666; }
  h1 { font-size:26pt; margin-bottom:14px; }
  h2 { color:#4f81bd; font-size:18pt; }
  p, li { font-size:15pt; line-height:1.35; }
  pre, code { font-size:11pt; }
  .small p, .small li { font-size:13pt; }
  .tiny p, .tiny li { font-size:11.5pt; }
  .flow { display:flex; align-items:center; gap:8px; margin:18px 0; }
  .box { background:#4f81bd; color:white; padding:12px 16px; border-radius:6px; font-size:12pt; font-weight:bold; text-align:center; flex:1; }
  .box.orange { background:#e67e22; }
  .box.green { background:#27ae60; }
  .arrow { font-size:22pt; color:#aaa; }
  .capture { border:2px dashed #888; border-radius:6px; padding:10px 12px; background:#fbfbfb; font-size:12.5pt; margin:8px 0; }
  table { width:100%; border-collapse:collapse; }
  td, th { border:1px solid #cccccc; padding:6px; font-size:12.5pt; }
  th { background:#4f81bd; color:white; }
---

# `Lecture 05: Gazebo Sensor Modeling`
## Camera, LiDAR, Ultrasonic, IPM BEV 확인

방산개방형아키텍처

---

# `1. 오늘의 목표`

오늘은 차량 모델에 센서를 붙이고, Gazebo/실차 센서 topic을 확인한다.

- `sensors.urdf.xacro`에서 camera, LiDAR, ultrasonic 모델을 읽는다.
- `sensor_config.yaml`에서 센서 장착 위치를 바꿔본다.
- Gazebo sensor topic이 ROS topic으로 bridge되는 과정을 확인한다.
- USB Camera와 RPLiDAR 실차 장치 확인 방법을 익힌다.
- IPM BEV node가 camera와 TF를 이용해 bird-eye-view를 만드는 흐름을 본다.

---

# `2. 오늘의 목표(Cont')`

<div class="flow">
  <div class="box">URDF/Xacro<br>sensor links</div>
  <div class="arrow">→</div>
  <div class="box orange">Gazebo<br>sensor plugin</div>
  <div class="arrow">→</div>
  <div class="box">ros_gz_bridge<br>ROS topics</div>
  <div class="arrow">→</div>
  <div class="box green">RViz/rqt<br>IPM BEV</div>
</div>

핵심 질문:

```text
센서 위치와 방향을 바꾸면, ROS topic과 TF에서 무엇이 달라지는가?
```

---

# `3. 실습 확인 방식`

시험/평가가 아니라 실습 체크포인트로 확인한다.

오늘 확인할 것:

```text
/camera_front/image_raw
/camera_rear/image_raw
/camera_info
/scan
/ultrasonic_front/scan
/ipm_bev/debug
TF: base_link -> camera_front_link, lidar_link
```

---

# `4. 센서 파일 위치`

```text
system/aac_vehicle/urdf/sensors.urdf.xacro
system/aac_vehicle/config/sensor_config.yaml
system/aac_vehicle/launch/spawn_vehicle.launch.py
perception/ipm_bev_node/config/ipm_bev.yaml
sensors/usb_cam/config/params_1.yaml
sensors/rplidar_ros2/launch/rplidar_a3.launch.py
```

Gazebo 센서와 실차 센서를 같은 ROS topic 흐름으로 비교한다.

---

# `5. Build`

```bash
# 실행 위치: AAC Container
cd /ros_ws
src/aac_2026/perception/setup_colcon_ignores.sh
colcon build --base-paths src/* --packages-select aac_vehicle ipm_bev_node usb_cam rplidar_ros
source install/setup.bash
```

패키지 확인:

```bash
# 실행 위치: AAC Container
ros2 pkg list | grep -E "aac_vehicle|ipm_bev_node|usb_cam|rplidar"
```

---

# `6. Gazebo 센서 실행`

```bash
# 실행 위치: AAC Container
source /ros_ws/install/setup.bash
ros2 launch aac_vehicle spawn_vehicle.launch.py
```

다른 terminal:

```bash
# 실행 위치: Host Ubuntu에서 시작, 이후 AAC Container
aac_enter
# aac_enter 이후부터는 AAC Container
ros2 topic list | grep -E "camera|scan|ultrasonic"
```

---

# `7. sensor_config.yaml`

```yaml
lidar:
  xyz: [0.0, 0.0, 0.33]
  rpy: [0.0, 0.0, 0.0]

camera_front:
  xyz: [0.45, 0.0, 0.70]
  rpy: [0.0, 0.2617, 0.0]
```

좌표:

```text
x+: forward, y+: left, z+: up
rpy: roll, pitch, yaw [rad]
```

---

# `8. Launch에서 sensor_config 읽기`

`spawn_vehicle.launch.py`

```python
sensor_config_file = os.path.join(pkg_share, 'config', 'sensor_config.yaml')
with open(sensor_config_file, 'r') as f:
    sensor_cfg = yaml.safe_load(f)

sensor_mappings[f'{sensor}_x'] = str(x)
sensor_mappings[f'{sensor}_yaw'] = str(yaw)
```

YAML 값이 Xacro argument로 들어간다.

```text
sensor_config.yaml -> xacro mappings -> robot_description
```

---

# `9. sensors.urdf.xacro 구조`

```text
lidar_link
camera_front_link
camera_rear_link
ultrasonic_front_link
ultrasonic_rear_link
ultrasonic_left_link
ultrasonic_right_link
```

각 link는 `base_link`에 fixed joint로 붙는다.

센서 장착 위치는 link의 `<origin xyz="..." rpy="...">`로 결정된다.

---

# `10. LiDAR 모델`

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
    <range>
      <min>0.15</min>
      <max>12.0</max>
    </range>
  </lidar>
</sensor>
```

---

# `11. LiDAR Topic 확인`

Gazebo bridge는 `/lidar/scan`을 `/scan`으로 remap한다.

```bash
# 실행 위치: AAC Container
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 interface show sensor_msgs/msg/LaserScan
```

관찰:

- `angle_min`, `angle_max`
- `angle_increment`
- `range_min`, `range_max`
- `ranges`
- `header.frame_id`

---

# `12. RViz에서 LaserScan 보기`

```bash
# 실행 위치: AAC Container
rviz2
```

RViz 설정:

```text
Fixed Frame: map 또는 odom
Add -> LaserScan
Topic: /scan
```

TF가 안 맞으면 `header.frame_id`와 TF tree를 확인한다.

---

# `13. Camera 모델`

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

`1.2217 rad`는 약 70도 HFoV다.

---

# `14. Camera Topic 확인`

```bash
# 실행 위치: AAC Container
ros2 topic echo /camera_front/image_raw --once --field header
ros2 topic hz /camera_front/image_raw
ros2 topic echo /camera_info --once
```

영상 보기:

```bash
# 실행 위치: AAC Container
rqt_image_view /camera_front/image_raw
```

---

# `15. Rear Camera`

rear camera는 뒤를 보도록 yaw가 약 pi로 설정되어 있다.

```yaml
camera_rear:
  xyz: [-0.45, 0.0, 0.70]
  rpy: [0.0, 0.2617, 3.14159]
```

확인:

```bash
# 실행 위치: AAC Container
rqt_image_view /camera_rear/image_raw
```

앞/뒤 방향이 맞는지 트랙 texture를 보며 확인한다.

---

# `16. Ultrasonic 모델`

초음파 센서는 Gazebo에서 narrow-beam `gpu_lidar`로 모델링한다.

```xml
<samples>1</samples>
<min_angle>-0.2618</min_angle>
<max_angle>0.2618</max_angle>
<range>
  <min>0.02</min>
  <max>4.0</max>
</range>
```

출력은 `LaserScan`이고 `ranges[0]`이 거리다.

---

# `17. Ultrasonic 확인`

```bash
# 실행 위치: AAC Container
ros2 topic echo /ultrasonic_front/scan --once
ros2 topic echo /ultrasonic_left/scan --once
ros2 topic hz /ultrasonic_front/scan
```

해석:

```text
ranges 배열 길이가 1이면 단일 beam 거리 센서처럼 볼 수 있다.
```

---

# `18. Bridge 설정`

`spawn_vehicle.launch.py`

```python
'/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
'/camera_front@sensor_msgs/msg/Image[ignition.msgs.Image',
'/camera_rear@sensor_msgs/msg/Image[ignition.msgs.Image',
'/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
'/ultrasonic_front/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
```

Gazebo sensor output을 ROS topic으로 넘긴다.

---

# `19. TF 확인`

```bash
# 실행 위치: AAC Container
ros2 run tf2_ros tf2_echo base_link camera_front_link
```

다른 센서:

```bash
# 실행 위치: AAC Container
ros2 run tf2_ros tf2_echo base_link lidar_link
```

확인:

```text
translation이 sensor_config.yaml의 xyz와 맞는가?
rotation이 rpy 설정과 맞는가?
```

---

# `20. Sensor 위치 변경 실습`

front camera 높이를 낮춰본다.

```yaml
camera_front:
  xyz: [0.45, 0.0, 0.50]
  rpy: [0.0, 0.2617, 0.0]
```

수정 후:

```bash
# 실행 위치: AAC Container
cd /ros_ws
colcon build --base-paths src/* --packages-select aac_vehicle
source install/setup.bash
ros2 launch aac_vehicle spawn_vehicle.launch.py
```

---

# `21. IPM BEV 개념`

IPM은 camera 영상을 지면 평면으로 역투영해 bird-eye-view를 만든다.

```text
camera image + CameraInfo + TF + ground plane
  -> BEV image
```

AAC의 출력:

```text
/ipm_bev/front
/ipm_bev/rear
/ipm_bev/fused
/ipm_bev/debug
```

---

# `22. IPM BEV 실행`

Gazebo 차량이 먼저 실행되어 있어야 한다.

```bash
# 실행 위치: AAC Container
source /ros_ws/install/setup.bash
ros2 launch ipm_bev_node deploy.launch.py
```

확인:

```bash
# 실행 위치: AAC Container
ros2 topic hz /ipm_bev/fused
rqt_image_view /ipm_bev/debug
```

---

# `23. ipm_bev.yaml`

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

BEV 영역과 해상도를 여기서 바꾼다.

---

# `24. BEV 해상도 계산`

기본값:

```text
x range = 20 m
y range = 12 m
resolution = 0.025 m/pixel
```

출력 크기:

```text
height = 20 / 0.025 = 800 px
width  = 12 / 0.025 = 480 px
```

해상도를 낮추면 빠르지만 거칠다.

---

# `25. CameraInfo fallback`

Gazebo bridge는 공용 `/camera_info`를 제공한다.

```yaml
use_camera_info: true
allow_fov_fallback: true
horizontal_fov: 1.2217
```

실차에서는:

```text
전방/후방 camera_info를 분리하고 실제 calibration 결과를 쓰는 것이 좋다.
```

---

# `26. 전체 Stack으로 실행`

```bash
# 실행 위치: AAC Container
cd /ros_ws/src/aac_2026
./start.sh
```

tmux에서 실행되는 sensor 관련 pane:

```text
Gazebo vehicle spawn
IPM BEV node
Image matching node
SLAM mapping node
```

오늘은 sensor와 IPM pane을 집중적으로 본다.

---

# `27. USB Camera 실차 확인`

Host에서 장치:

```bash
# 실행 위치: Host Ubuntu
ls /dev/video*
v4l2-ctl --list-devices
```

Container에서 launch:

```bash
# 실행 위치: AAC Container
ros2 launch usb_cam camera.launch.py
```

영상 확인:

```bash
# 실행 위치: AAC Container
rqt_image_view /camera1/image_raw
```

---

# `28. USB Camera 설정`

`sensors/usb_cam/config/params_1.yaml`

```yaml
video_device: "/dev/video0"
framerate: 30.0
io_method: "mmap"
frame_id: "camera"
pixel_format: "mjpeg2rgb"
image_width: 640
image_height: 480
camera_info_url: "package://usb_cam/config/camera_info.yaml"
```

장치 번호가 다르면 `video_device`를 수정한다.

---

# `29. 두 번째 Camera`

`params_2.yaml`

```yaml
video_device: "/dev/video2"
framerate: 15.0
frame_id: "camera2"
image_width: 640
image_height: 480
```

확인:

```bash
# 실행 위치: AAC Container
ros2 topic list | grep camera2
rqt_image_view /camera2/image_raw
```

---

# `30. RPLiDAR 실차 확인`

Host에서 serial device:

```bash
# 실행 위치: Host Ubuntu
ls /dev/ttyUSB*
dmesg | grep tty | tail -10
```

Container에서:

```bash
# 실행 위치: AAC Container
ros2 launch rplidar_ros rplidar_a3.launch.py
```

---

# `31. RPLiDAR 확인`

```bash
# 실행 위치: AAC Container
ros2 topic hz /scan
ros2 topic echo /scan --once
rviz2
```

RPLiDAR A3 launch 기본값:

```text
serial_port: /dev/ttyUSB0
serial_baudrate: 256000
frame_id: lidar_link
scan_mode: Sensitivity
```

---

# `32. udev rule`

USB serial 번호는 재부팅이나 재연결 때 바뀔 수 있다.

RPLiDAR rule:

```text
sensors/rplidar_ros2/scripts/rplidar.rules
```

설치:

```bash
# 실행 위치: Host Ubuntu
cd /home/${USER}/ros_ws/src/aac_2026/sensors/rplidar_ros2/scripts
sudo ./create_udev_rules.sh
```

---

# `33. Docker device 전달`

`run_x86.sh`는 다음 장치를 container에 전달한다.

```text
/dev/video*
/dev/input/js*
/dev/ttyUSB*
/dev/ttyACM*
/dev/dri
```

확인:

```bash
# 실행 위치: Host Ubuntu
cd /home/${USER}/ros_ws/src/aac_2026
grep -n "V4L2_DEVICES\\|USB_DEVICES\\|ACM_DEVICES\\|JS_DEVICES" docker/run_x86.sh
```

---

# `34. Gazebo 센서와 실차 센서 비교`

| 항목 | Gazebo | 실차 |
|---|---|---|
| Camera | `/camera_front/image_raw` | `/camera1/image_raw` |
| LiDAR | `/scan` | `/scan` |
| TF | URDF/Xacro | static TF 또는 robot_state |
| Calibration | 설정값 | 실제 calibration |
| Noise | 모델링 값 | 실제 환경 |

실차 topic 이름은 후속 통합 단계에서 remap하거나 config로 맞춘다.

---

# `35. 흔한 실패`

| 증상 | 확인 |
|---|---|
| camera topic 없음 | bridge, Gazebo sensor, `/camera_front` remap |
| rqt가 검은 화면 | Gazebo rendering, camera 방향, topic 선택 |
| `/scan` 없음 | bridge, lidar sensor, frame_id |
| 실차 camera 없음 | `/dev/video*`, config `video_device` |
| 실차 LiDAR 없음 | `/dev/ttyUSB*`, baudrate, udev |
| IPM 결과가 뒤집힘 | camera TF, `camera_frame_convention`, pitch/yaw |

---

# `36. 디버깅 순서`

```text
1. 센서 장치 또는 Gazebo sensor가 존재하는가?
2. ROS topic이 publish되는가?
3. topic type이 맞는가?
4. header.frame_id가 TF tree에 있는가?
5. update rate가 기대와 비슷한가?
6. CameraInfo 또는 intrinsic fallback이 있는가?
7. downstream node가 같은 topic 이름을 보고 있는가?
```

---

# `37. 캡처 후보`

<div class="capture">
<strong>[CAPTURE SLOT]</strong><br>
대상: Gazebo camera image와 `/ipm_bev/debug` 비교 화면<br>
확인: front/rear camera가 BEV로 합성되는 흐름을 보여준다.
</div>

<div class="capture">
<strong>[CAPTURE SLOT]</strong><br>
대상: RViz LaserScan 화면<br>
확인: `/scan`이 차량 주변 장애물/트랙 구조를 반영한다.
</div>

---

# `38. 영상 슬롯`

```text
[VIDEO SLOT]
목적: sensor_config.yaml에서 camera height/pitch를 바꿨을 때 IPM BEV가 달라지는 장면
추천 길이: 60-90초
삽입 위치: pptx export 후 수동 삽입
```

---

# `39. 실습 체크포인트`

```text
Gazebo camera topic을 확인했다.
Gazebo /scan topic을 확인했다.
sensor_config.yaml의 xyz/rpy 의미를 설명했다.
TF echo로 camera/lidar 위치를 확인했다.
IPM BEV debug image를 열었다.
USB Camera 또는 RPLiDAR 장치 확인 명령을 실행했다.
```

---

# `40. 다음 강의로 연결`

다음 강의에서는 `/ipm_bev/fused`를 전역 지도 이미지와 매칭한다.

```text
camera_front/rear
  -> ipm_bev_node
  -> /ipm_bev/fused
  -> image_match_node
  -> /visual_localization/opencv/pose
```

센서 모델은 항법 알고리즘의 입력 품질을 결정한다.

---

# `41. 참고 문서`

- Gazebo Sensors: https://gazebosim.org/libs/sensors/
- Gazebo sensor tutorial: https://gazebosim.org/docs/latest/sensors/
- SDFormat sensor spec: https://sdformat.org/spec/1.12/sensor/
- ROS 2 usb_cam: https://github.com/ros-drivers/usb_cam
- RPLiDAR SDK: https://github.com/Slamtec/rplidar_sdk
- Project files: `system/aac_vehicle/urdf/sensors.urdf.xacro`, `perception/ipm_bev_node`

