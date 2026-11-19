# 3D 위치 추정과 MPCC 입력

## 젯슨에서 PS5 수동주행과 MPCC 기록 시작

저장소에서 관리하는 새 통합 진입점은 [실차 tmuxp 실행](../../operations/tmuxp_config/README.md)의 `bash start.sh prepare`입니다. 아래의 기존 배치 스크립트와 동시에 시작하지 않습니다. `prepare`는 MPCC 계산·기록과 수동주행 구성이며, 실제 MPCC 자율주행 출력 연결은 별도입니다.

실내 장비에 배치된 통합 명령으로 라이다, 지도 위치 추정, MPCC 계산, 센서와 PS5 기록을 tmux에 올립니다. 준비가 끝나면 `Ready`가 출력됩니다.

```bash
cd ~/f1tenth_ws/live_indoor
./indoor_tmux.sh start
./indoor_tmux.sh status
./indoor_tmux.sh stop
```

세션 `aac-drive-check`의 `mpcc`, `sensors`, `ps5` 창에서 진행 상태를 봅니다. `stop`은 이 세 창의 기록을 마감합니다. 매 실행은 젯슨의 `~/f1tenth_ws/live_indoor/runs/` 아래 새 폴더에 저장합니다. `sensors/`에는 MID-360과 위치 추정, `ps5/`에는 수동 입력과 VESC 피드백, `mpcc.csv`에는 각 입력 시각과 계산 결과 또는 제외 이유가 들어갑니다. CSV의 `drive_normalized`는 모델의 정규화 구동 입력이고 `steering_rad`는 조향각입니다. 실제 주행은 PS5 수동 명령으로 합니다.

저장을 마치면 VESC 휠 속도로 첫 움직임과 마지막 정차를 찾아, 앞뒤로 최대 5초를 포함한 사본을 `trim5s/`에 저장합니다. 두 rosbag과 CSV에 같은 시간 범위를 적용하고 재생에 필요한 정적 TF도 복사합니다. 휠 속도를 확인할 수 없는 구간과 주행 중간 정차는 보존합니다. 기록이 끝난 기존 실행에는 `./indoor_tmux.sh trim runs/실행폴더`를 사용합니다.

## 실차 rosbag으로 복도 위치 추정 재생

MID-360의 `/livox/lidar`와 `/livox/imu`를 FAST-LIO에 전달합니다. 위치 추정 노드는 같은 시각의 FAST-LIO 자세와 움직임이 보정된 점군을 받아 기존 복도 지도에 정합합니다. 녹화된 `base_link → livox_frame` TF와 MID-360 내부 IMU 보정을 적용해 차체 중심의 위치·자세·속도를 `/localization/odom`에 냅니다. MPCC 입력 노드는 같은 시각의 상태를 MINCURV 경로에 투영해 `/mpcc/state`로 발행합니다.

| 토픽 | 확인할 값 |
|---|---|
| `/localization/odom` | `indoor_map` 좌표계의 `base_link` 자세와 차체 좌표계 속도 |
| `/mpcc/state` | `[x, y, yaw, vx, vy, yaw_rate, s]`. 거리 단위는 m, 각도는 rad이고 `s`는 폐회로의 시작선을 지나도 이어지는 경로 진행 거리입니다. |

본체의 저장된 bag, 지도와 경로를 사용하는 예시입니다. 새 메시지와 SciPy가 포함되도록 본체에서 환경을 한 번 빌드하고 셸에 들어갑니다.

```bash
cd ~/ros2_ws/src/aac_2026
bash navigation/fastlio/scripts/slam.sh build
bash navigation/fastlio/scripts/slam.sh shell
```

열린 컨테이너 셸에서 다음 명령을 실행합니다. 초기 자세는 2026-11-19 녹화의 출발점에 맞춘 `[x,y,z,roll,pitch,yaw]`입니다. 다른 출발점에서 사용할 때는 해당 위치와 방향을 지정하세요.

```bash
ros2 launch /ws/src/aac_2026/navigation/localization3d/localization_mpcc.launch.py \
  map_dir:=/ws/tracks/indoor_final_cleaned \
  track_csv:=/ws/tracks/indoor_final_cleaned_mincurv_asymmetric/track.csv \
  'initial_base_pose:=[-0.802975,12.109757,-0.824311,0.047162,-0.000567,-1.706791]' \
  use_sim_time:=true &
pipeline=$!
```

`Prior loaded`와 `Projecting /localization/odom`가 뜨면 **같은 셸에서** 다음 명령을 실행합니다. 첫 상태 메시지를 화면에 표시한 후에도 재생은 끝까지 계속됩니다.

```bash
ros2 topic echo /mpcc/state --once &
ros2 bag play /ws/bag/indoor_newcar_20261119_032653_trim10s --clock 100 --rate 1.0
kill -INT "$pipeline"
wait "$pipeline"
```

각 재생은 위치 추정 노드와 FAST-LIO를 새로 시작합니다. 재생 중 ROS 시간은 bag 플레이어가 공급합니다. 실제 MID-360을 직접 받을 때는 `use_sim_time:=false`를 사용하고, 현재 차량의 장착 TF와 지도상의 초기 자세를 공급합니다. 이 launch의 출력은 MPCC 상태 토픽까지입니다.

### 라이다로 초기 자세 검색

위 launch의 `initial_base_pose`를 `auto`로 바꾸면 자동 초기화를 실행합니다. 지도 폴더에는 `map.pcd`, `map_evidence.npz`, `occupancy.yaml`이 필요합니다. 젯슨의 tmuxp 구성에서는 `AAC_INITIAL_BASE_POSE=auto`에 해당합니다.

3D 점군을 지도에 비교해 한 층 전체에서 `x, y, yaw`를 찾습니다. 높이는 지도 바닥면과 실측 차축 높이 5.5cm로 잡고, roll과 pitch는 FAST-LIO를 사용합니다. 서로 떨어진 후보를 최대 16개 유지하고 후보당 32개 입자를 FAST-LIO 이동량으로 갱신합니다. 연속된 키프레임에서 같은 후보가 우세하고 3D ICP까지 통과하면 `Global hypothesis confirmed by 3D ICP`가 표시됩니다. 그 다음부터 기존 추적기가 `/localization/odom`을 발행합니다.

복도 후보들이 비슷하면 `Localization` 창에서 후보 수, 점수 차이와 확정 진행 상황을 확인할 수 있습니다. 새 구조물이 보이는 구간의 관측이 들어올 때까지 확정을 보류합니다. `prepare` 모드에서는 PS5 수동주행으로 관측을 추가할 수 있습니다.

### 같은 재생에서 MPCC 계산 확인

본체에 설치한 acados 환경까지 사용할 때는 위의 셸을 `bash navigation/fastlio/scripts/slam.sh shell --acados`로 엽니다. 위치 추정 launch를 띄운 같은 셸에서 bag을 재생하기 전에 아래 명령을 실행합니다.

```bash
PYTHONPATH="$ACADOS_SOURCE_DIR/interfaces/acados_template:$PYTHONPATH:/usr/lib/python3/dist-packages" \
  bash /ws/src/aac_2026/control/acados_mpcc/scripts/acados_python.sh \
  -m control.acados_mpcc.ros_shadow --ros-args -p use_sim_time:=true \
  -p track_csv:=/ws/tracks/indoor_final_cleaned_mincurv_asymmetric/track.csv &
shadow=$!
```

`Shadow solver ready`가 뜨면 위의 bag 재생 명령을 실행합니다. 이 노드는 새 상태마다 기존 실내 Slash 모델과 경로의 `soft_profile.npz`로 해를 계산하고, 종료 시 수신·통과 건수와 제외 사유를 출력합니다. 이전 제어 입력은 0으로 시작해 직전 계산값으로 갱신합니다. 출력은 로그에만 기록합니다. 재생이 끝난 뒤 `kill -TERM "$shadow"`와 `wait "$shadow"`로 종료합니다.

## 가상 센서를 사용한 기존 오프라인 실험

아래 실험은 가상 3D 점군을 고정 지도에 ICP로 정합하고, 바퀴 속도·IMU 자이로로 센서 사이 자세를 예측합니다. 추정 결과가 MPCC 입력으로 들어갑니다.

| 위치 | 역할 |
|---|---|
| `inputs/` | 하나로 공유하는 지도·경로·영역 입력과 원본 해시 |
| `voxel_sensor.py`, `measurements.py` | 가상 측정 생성 |
| `estimator.py`, `feedback.py` | 3D 추정·제어기 상태 변환 |
| `closed_loop.py`, `soft_runtime.py`, `solver_margin.py` | 오프라인 폐루프·solver 연결 |
| `audit.py`, `geometry.py` | 기록된 결과의 독립 영역·오차 검사 |
| `display.py`, `display_assets/` | GT·오도메트리·추정 위치의 기록 재생 |
| `prepare.py` | 원본 점군으로 별도 입력 묶음 준비 |
| `results/` | 새 실험 결과; Git 제외 |

본체에서:

```bash
cd ~/ros2_ws/src/aac_2026
bash operations/f1tenth/run.sh localization-test

cd navigation/localization3d
bash ../../operations/f1tenth/run.sh python -W ignore::DeprecationWarning \
  "$PWD/closed_loop.py" --steps 7000
```

새 실행이 출력한 결과 경로를 `RUN`에 넣어 검사합니다. `RUN`은 절대 경로를 사용합니다.

```bash
RUN=/absolute/path/to/localization3d/results/RUN_ID
bash ../../operations/f1tenth/run.sh python "$PWD/audit.py" --run "$RUN"
bash ../../operations/f1tenth/run.sh python "$PWD/display.py" --run "$RUN"
```

표시 결과는 기록 재생이며 Gazebo 접촉 물리나 실시간 센서·제어 검증은 아닙니다. 월드의 점군 메시를 읽으려면 본체의 관련 `results/` 자료와 표시 plugin이 추가로 필요합니다.

입력은 이미 포함돼 있습니다. 다시 준비할 경우 기존 입력을 덮어쓰지 않고 새 출력 폴더를 지정합니다. 본체의 원본 점군과 계획 격자가 있어야 합니다.

```bash
bash ../../operations/f1tenth/run.sh python "$PWD/prepare.py" \
  --output /absolute/path/to/new-inputs
```

2026-11-04 PC·Nano 기준 실험에서는 가상 한 바퀴와 3D 추정 상태의 MPCC 전달을 확인했습니다. 같은 voxel 지도·센서 월드를 사용한 유리한 조건이며 실측 정확도를 뜻하지 않습니다. Nano의 전체 반복 평균은 약 74ms로 20ms 실시간 제어를 입증하지 못했습니다. 센서·solver 가정과 모든 기존 입력 값·해시는 유지했습니다.

기존 전체 실험 설명은 본체 `log/f1tenth/cleanup-20261109/before/navigation/localization3d/README.md`, 과거 결과는 `operations/f1tenth/archive/localization_results`에 보존했습니다. AGX 배포·실측 센서 보정·실차 연결은 별도 작업입니다.
