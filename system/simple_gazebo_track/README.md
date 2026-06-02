# Simple Gazebo Track

업로드한 트랙 이미지를 12 m × 16 m 평면 텍스처로 올린 가장 단순한 Gazebo Sim / Ignition 환경입니다.

## 실행 방법

```bash
cd /ros_ws
colcon build --packages-select simple_gazebo_track aac_vehicle
source /ros_ws/install/setup.bash
ros2 launch aac_vehicle spawn_vehicle.launch.py
```

## 좌표계

- 트랙 중심: `(0, 0)`
- 크기: `12 m × 16 m`
- 좌하단 대략: `(-6, -8)`
- 우상단 대략: `(6, 8)`

## 수정 포인트

트랙 실제 크기를 바꾸려면 아래 두 파일의 box `size` 값을 같은 값으로 수정하세요.

- `models/simple_track/model.sdf`
- `worlds/simple_track.world`

차량 spawn 위치 예시:

```xml
<pose>-7.0 -0.5 0.1 0 0 1.5708</pose>
```

이 버전은 가장 단순한 이미지 기반 바닥 모델입니다. 차선/벽/장애물에 실제 collision을 주려면 추후 SDF box/mesh를 추가하면 됩니다.
