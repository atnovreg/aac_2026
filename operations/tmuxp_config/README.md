# Jetson 실행

원래 `start.sh`와 `auto.sh`는 `config.yaml`의 Gazebo·가상 오도메트리·주차 구성을 실행했다. 그 구성은 `simulation.yaml`로 옮겼다. 이제 `config.yaml`은 Jetson 실차용이며, 실행 환경을 전달하는 루트 `start.sh`를 통해 실행한다.

## 현재 실행할 수 있는 범위

`prepare`는 실제 MID-360, 장착 TF, FAST-LIO, 지도 위치추정, MPCC 상태 입력, MPCC 계산, PS5 수동주행과 기록을 실행한다. MPCC가 차량을 운전하지는 않는다. 실제 젯슨에서 확인한 기존 설정을 사용한다.

`autonomy`는 실내 프로필 또는 두 개의 별도 실행 스크립트가 필요하다. 기본 실내 프로필은 MPCC가 0.1초 뒤에 예측한 전진 속도를 VESC의 속도 목표로 전달한다. 조향은 현재 MPCC 계산값을 서보 단위로 환산한다. `drive_normalized`는 MPCC 내부와 로그에 사용한다.

## 설정과 실행

젯슨의 저장소 루트에서 `operations/tmuxp_config/jetson.env`를 편집한다. 기본 경로는 `~/f1tenth_ws/live_indoor`, ROS는 Humble이다. 경로를 바꿀 때는 절대 경로를 사용한다. `AAC_INITIAL_BASE_POSE=auto`를 지정하면 3D 라이다로 초기 자세를 검색한다. 지도 폴더에 `occupancy.yaml`이 추가로 필요하다. 수동 초기화 값은 현재 차량의 지도 기준 `[x,y,z,roll,pitch,yaw]`이며 단위는 m/rad이다.

```bash
cd ~/f1tenth_ws/src/aac_2026

# 초기 자세 설정 후 파일·실행 중복 확인. 노드는 켜지지 않는다.
bash start.sh check prepare

# 센서·위치추정·수동주행·MPCC 계산/기록
bash start.sh prepare
# SSH에서 화면을 붙이지 않고 실행하려면:
bash start.sh prepare -d
tmux attach -t AAC_2026

# 해당 세션의 노드와 기록을 함께 종료
bash stop.sh
```

tmux 화면에서 빠져나오면서 계속 실행하려면 `Ctrl-b` 다음 `d`를 누른다. `Vehicle`, `Lidar`, `Localization`, `MPCC`, `Record`, `Monitor` 창이 생긴다. 모니터는 `/mpcc/state` 발행률을 표시한다. 창이 만들어졌거나 설정 검사가 통과했다고 위치추정·제어 준비가 완료된 것은 아니다. 실패한 pane은 종료 상태와 오류를 남긴다.

기존 `aac-manual`, `aac-lidar`, `aac-localization`, `aac-drive-check` 등에서 같은 노드를 실행 중이면 새 시작이 중단된다. `indoor_tmux.sh stop`은 기록과 MPCC만 멈추므로 차량·센서 프로세스가 남아 있을 수 있다. 기존 세션의 상태를 확인하고 해당 실행의 원래 종료 방법으로 정리한다. 새 `stop.sh`는 `AAC_2026`만 정리하며 다른 세션이나 ROS 프로세스를 일괄 종료하지 않는다.

기록은 매번 `~/f1tenth_ws/live_indoor/runs/날짜_고유번호/`에 생성된다. `sensors/`, `ps5/`, `mpcc.csv`와 각 pane의 `.log`, 실행 당시의 `settings.env`가 남는다. `prepare`의 센서·MPCC는 domain 43, 수동주행·PS5 녹화는 기존처럼 domain 42이다. 실차 센서·위치추정·MPCC 시간은 `use_sim_time=false`이다. 녹화의 자동 trim은 실행하지 않는다.

종료가 15초 안에 완료되지 않으면 세션을 유지하고 실패를 표시한다. 남은 pane을 확인하고 `STOP_WAIT_SECONDS=60 bash stop.sh`로 더 기다릴 수 있다. 세션이 없어졌다는 것만으로 파일 저장이 정상 완료됐다고 가정하지 말고 rosbag의 `metadata.yaml`과 로그를 확인한다.

## 실내 단독 주행

실내 프로필은 자율 목표 속도를 2.2m/s, VESC 출력 명령을 ±8,300 ERPM으로 제한한다. 속도·조향 환산은 2026-11-19 두 바퀴 주행의 VESC 기록과 FAST-LIO 상태를 비교한 추정값이다. 이 값은 차를 띄운 상태에서 바퀴 방향과 정지 동작을 별도로 확인할 때 사용한다.

```bash
export AAC_INITIAL_BASE_POSE=auto
export AAC_AUTONOMY_PROFILE="$PWD/system/f1tenth/config/agx/indoor_drive.yaml"
bash start.sh check autonomy
bash start.sh autonomy
```

이 구성은 `/drive/preview`에만 구동 명령을 낸다. 실제 VESC에 연결하려면 정지·방향·워치독을 확인할 수 있는 실차 환경에서 `AAC_AUTONOMY_HARDWARE=1`을 명시한다. preview는 격리된 ROS 세션의 VESC 피드백 또는 재생 데이터로 명령을 확인한다.

PS5에서 모든 버튼을 놓고 ○로 준비한다. R1을 누른 채 수동으로 움직여 전역 위치가 잡히고 MPCC가 유효한 결과를 내도록 한다. R1을 놓고 L1을 새로 누른 채 유지하면 자율 출력을 선택한다. R1을 누르면 수동이 우선이고, L1을 놓거나 ×를 누르면 정지 명령을 보낸다. 위치 입력·MPCC 요청이 낡았거나 VESC 응답이 끊기면 정지한다. 자율 요청이 거절된 뒤에는 L1을 놓았다가 다시 눌러야 한다. 피드백이 끊겼거나 ×를 눌렀다면 ○부터 다시 시작한다.

ROS에서 보내는 정지는 VESC 속도 0 명령이다. 선택기 또는 USB 연결 자체가 사라진 경우에는 VESC 펌웨어의 타임아웃 설정에 의존한다. 주행 시험에서는 물리적인 정지 거리를 확인해야 한다.

다른 제어 구성은 `AAC_AUTONOMY_PROFILE` 대신 `jetson.env`의 두 실행 파일로 지정한다. 스크립트는 마지막에 장기 실행 프로세스를 `exec`하여 tmux의 종료 신호가 전달되게 한다.

- `AAC_CONTROL_START`: `/mpcc/state`를 받아 계산하고, 보정된 실제 구동·제동·조향 명령을 선택기에 전달하는 제어 실행 파일. 오래된 상태와 solver 실패 시 명령을 중단해야 한다.
- `AAC_VEHICLE_START`: VESC와 PS5 수동/자동 선택기 실행 파일. 시작 시 비활성 상태이며 수동 개입·정지·입력 끊김을 처리해야 한다. 현재 수동 전용 launch를 그대로 지정하면 자율 명령을 받지 않는다.

두 실행 파일은 센서·위치추정과 같은 domain 43 및 `AAC_TRACK_CSV`, `AAC_RUN_DIR` 환경을 받는다. 모터 설정을 새로 선택하거나 보정하는 작업은 이 실행 스크립트가 대신하지 않는다.

## 기존 시뮬레이션

기존 Humble 컨테이너(`/ros_ws`)에서:

```bash
bash start.sh sim
bash stop.sh
```

`auto.sh`도 같은 인수를 지원한다. `kill_sys.sh`는 기존 Docker 컨테이너 제거용이므로 Jetson 네이티브 실행 종료에는 사용하지 않는다.
