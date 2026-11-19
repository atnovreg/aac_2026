# AGX MID-360 실차 장착

실측 기준점 `base_link`는 앞뒤 차축의 중간, 차량 중심선 위, 바닥에서 5.5 cm 높이에 있다.
+X는 차량 전방, +Y는 좌측, +Z는 위쪽이다.

| 부모 → 자식 | 이동 x, y, z (m) | yaw |
|---|---|---|
| `base_link → rear_axle` | −0.1675, 0, 0 | 0° |
| `base_link → lidar_vehicle` | +0.116, 0, +0.177 | 0° |
| `lidar_vehicle → livox_frame` | 0, 0, 0 | −90° |

roll과 pitch는 모두 0°다. 수평 실측은 센서 중심을 기준으로 사용했다.
점군 원점 높이는 센서 바닥면 실측 18.5 cm에 제조사 도면의 4.7 cm를 더한 23.2 cm다.
여기서 차축 높이 5.5 cm를 빼면 TF의 z가 17.7 cm다. 휠베이스 33.5 cm 기준으로
후륜 차축에서 점군 원점까지의 전방 거리는 28.35 cm다. [Livox 장착 도면, 5쪽](https://dl.djicdn.com/downloads/Livox/Mid-360/QSG/Livox_Mid-360_Quick_Start_Guide_multi.pdf#page=5)

젯슨 SSH 터미널에서 실행:

```bash
source /opt/ros/humble/setup.bash
source /home/irdlsm/f1tenth_ws/install/lidar/setup.bash
export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=1
ros2 launch /home/irdlsm/f1tenth_ws/src/aac_2026/system/f1tenth/config/agx/mid360.launch.py
```

디스플레이가 연결된 터미널에서는 `rviz:=true`를 붙이면 차량 방향으로 표시한다.
같은 ROS 환경의 다른 터미널에서 `ros2 run tf2_ros tf2_echo base_link livox_frame`으로
실측 기준의 합성 TF를 확인할 수 있다.
기존 드라이버와 중복 실행하지 않는다. 이 launch는 수동 주행 프로그램과 별개이다.
현장 확인한 주소는 host `192.168.1.50`, MID-360 `192.168.1.107`이며
`MID360_config.json`에서 관리한다. `profile.yaml`은 이전 조사 기록이다.

`/livox/lidar`와 `/livox/imu`는 원본 센서 좌표를 유지한다. 차량 방향 좌표가 필요한
소비자는 TF로 `lidar_vehicle`에 변환해야 한다. RViz Fixed Frame도 이 프레임이다.
드라이버 JSON의 extrinsic은 0으로 유지해 이중 회전을 방지한다.
FAST-LIO의 LiDAR-IMU 내부 extrinsic에는 이 장착 회전을 넣지 않는다.
CustomMsg가 필요한 경우 `xfer_format:=1`을 사용한다(RViz PointCloud2 표시는 별도 변환 필요).
기존 전방 180도 필터를 붙일 때에는 원본 센서 좌표 기준 `front_yaw_deg:=90.0`을
지정해야 차량 앞쪽을 선택한다. 이전 Nano/녹화 데이터의 정면 설정과 혼용하지 않는다.
