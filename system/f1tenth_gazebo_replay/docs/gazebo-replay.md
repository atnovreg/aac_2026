# 실측 3D 지도 위 acados 주행 재생

```bash
./scripts/gazebo_replay.sh build   # 이미지가 없을 때 최초 한 번
./scripts/gazebo_replay.sh
```

Gazebo Harmonic 창에서 `tracks/indoor_final/map.pcd`의 실제 XYZ 점과 저장한 acados 주행을 함께 표시한다. 최신 완주 SQP_RTI 결과를 자동 선택하며, 한 바퀴 끝에서 2초 유지한 뒤 반복한다. 재생 시간은 Gazebo 시뮬레이션 시간 기준이다. 좌측 재생/일시정지 버튼으로 멈출 수 있고 마우스로 시점을 바꿀 수 있다.

- 차체는 현재 모델인 ORCA 크기(길이 12cm, 폭 6cm)다. 빨간 구와 흰 방향 막대는 잘 보이도록 차량 위 2.5m에 둔 위치/방향 표시다.
- 녹색/주황색 선은 MPCC 좌우 제약 경계, 파란 선은 중심선이다. PCD는 벽·바닥 등을 포함한 원본 센서 관측점이며, 이동 물체의 잔상이 포함될 수 있다.
- 원본 약 2,471만 점은 수정하지 않는다. 표시용으로 최대 약 200만 점을 먼저 샘플링하고 10cm voxel마다 원본 점 하나를 남긴다. 최대 12만 점이며 현재 지도는 66,524개다. 점은 7cm 크기 사면체로 그린다. 표면 복원이나 충돌용 mesh가 아니다.
- 위에서 내부가 보이도록 바닥 기준 -0.12~2m의 점을 표시한다. 높이 제한은 `--map-height 3` 등으로 바꿀 수 있다.
- 차량의 X/Y/yaw는 저장된 trajectory.csv를 시간 보간한다. Z는 트랙 추출 때 추정한 바닥 평면 `z=0.0350323259*x-0.0074175769*y-0.7699569332`에 차체 높이 0.035m를 더한다. 지도와 궤적의 XY는 기존 camera_init 좌표 그대로다. 차량 roll/pitch는 0이다.

이 기능은 **기록 재생**이다. Gazebo가 차량 동역학을 계산하거나 acados가 Gazebo 상태를 받아 다시 제어하지 않는다. 센서 시뮬레이션, 실시간 위치 추정, F1TENTH 모델, 차량용 ROS TF 연결은 포함하지 않는다.

```bash
./scripts/gazebo_replay.sh --speed 0.5       # 시뮬레이션 시간 기준 반속
./scripts/gazebo_replay.sh --map-height 3   # 더 높은 벽/천장까지 표시
./scripts/gazebo_replay.sh --headless       # GUI 없이 재생 서버
./scripts/gazebo_replay.sh --run /ws/results/acados_mpcc/20260910T132001206044Z
```

동시에 여러 재생 서버를 켜지 않는다. 현재 작업에서 띄운 창을 종료하려면 창을 닫거나 `docker stop f1tenth-gazebo-replay`를 사용한다. 스크립트로 실행한 경우 Ctrl+C로 종료한다. GUI에는 현재 데스크톱의 DISPLAY/XAUTHORITY를 사용한다.

생성물은 `results/gazebo_replay/<UTC>/`에 저장한다: world.sdf, 표시용 STL/OBJ, 재생 CSV, 원본 경로·해시·필터 설정 manifest.json. 소스는 `src/aac_2026/system/f1tenth_gazebo_replay/`, Docker 설정은 `src/aac_2026/docker/f1tenth/`에 있다.

검증: Python 지도 샘플링/세계 생성 테스트 3개, C++ 시간 보간·yaw wrap·잘못된 입력 거부 테스트 통과. 실제 Gazebo pose 토픽 23개를 기록 CSV 및 바닥 평면과 대조했고 최대 XYZ 오차 5e-16m, 표본 간 이동 1.736m를 확인했다. 근거: `results/gazebo_replay/20260910T141513401030Z/replay_validation.json` 및 pose_samples.jsonl.
