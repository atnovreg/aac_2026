# AAC에서 F1TENTH 작업하기

수정할 소스는 `~/ros2_ws/src/aac_2026` 하나입니다. 기존 AAC 노드와 분류를 유지하고 본체·Nano 작업을 담당 기능에 넣었습니다.

## 전체 위치

```text
~/ros2_ws/
├── src/aac_2026/     GitHub로 관리하는 코드·설정
├── bag/              녹화한 센서 데이터: bag/bags를 나누지 않음
├── tracks/           생성한 경로
├── results/          지도·실험 결과
├── build/            기존 AAC 빌드
├── install/          기존 AAC 설치
├── log/              기존 AAC 로그
└── artifacts/        별도 Jazzy/CPU 빌드·검증 기록
```

`vendor/acados`와 숨김 `.venv-*`는 본체의 기존 의존성입니다. 소스용 `tools/`, `scripts/`, `config/`, `requirements/` 별칭과 루트 `compose.yaml`은 제거했습니다. 설정과 실행 스크립트는 담당 기능 아래의 `config/`, `scripts/`를 사용합니다.

## AAC 안의 분류

| 위치 | 용도 |
|---|---|
| `sensors/livox_ros_driver2` | Nano의 실제 MID-360 드라이버 |
| `sensors/livox_front_filter` | 센서 전방 필터·RViz |
| `sensors/realsense-ros`, `realsense_capture` | RealSense 드라이버·촬영 도구 |
| `sensors/bag_adapter` | 과거 bag 입력 어댑터·점군 자료형 |
| `navigation/fastlio` | FAST-LIO와 bag 기반 매핑 |
| `navigation/lidar_filter`, `map_review` | 지도 처리·검토 |
| `navigation/localization3d` | 3D 위치 추정·MPCC 오프라인 실험 |
| `planning/geonji`, `track_pipeline` | 지도 기반 경로 생성·차체 여유 처리 |
| `control/acados_mpcc`, `mpcc_cpp_virtual` | MPCC 모델·solver, C++ 가상 차량 |
| `control/mpcc_tuner`, `run_analysis` | 가중치 GUI·결과 분석 |
| `system/f1tenth` | 본체·Nano·AGX 설정, 공유 경로 처리 |
| `system/f1tenth_gazebo_replay` | 기록 궤적의 Gazebo 재생 |
| `docker/f1tenth` | Jazzy 환경·보존한 Livox SDK |
| `operations/f1tenth` | 설치·검증·실행 보조·문서 |
| `operations/f1tenth/archive` | 예전 워크스페이스·CAD·별도 실험 위치 색인 |

기존 Pure Pursuit·Stanley·조이스틱 노드는 유지했습니다. MPCC와 기존 노드의 입출력·명령 선택 연결은 별도 통합 작업입니다. 기존 `start.sh`가 추가한 F1TENTH 전체를 자동 실행하지는 않습니다.

## 본체와 Jetson

소스는 같고 장비별 설정·빌드만 구분합니다.

| 작업 | 위치 |
|---|---|
| 코드 작성·녹화 데이터 검사·지도·경로·튜닝·분석 | 본체 |
| SDK·acados·ROS 패키지·solver 빌드 | 실행할 각 장비 |
| 실제 LiDAR·IMU·VESC 연결과 차량 실행 | 차량의 Jetson |

최종 차량 컴퓨터가 AGX라면 새 배포는 AGX에 하면 됩니다. Nano를 다시 설치할 필요는 없으며 AGX 전환을 검증하기 전까지 기존 Nano 환경을 보존합니다. 실제 AGX OS·JetPack·드라이버는 아직 확인하지 않았습니다.

기존 AAC는 Humble, 가져온 F1TENTH는 Jazzy 환경을 사용합니다. 서로의 `install/setup.bash`를 섞지 않습니다. F1TENTH ROS 산출물은 `artifacts/f1tenth/jazzy-x86_64/` 또는 `jazzy-aarch64/` 아래에서 구분합니다. `docker/f1tenth/compose.yaml`은 현재 본체용 구성이고 AGX 완성 배포 파일은 아닙니다.

## 실행

```bash
cd ~/ros2_ws/src/aac_2026
bash operations/f1tenth/run.sh check
bash operations/f1tenth/run.sh python -m control.acados_mpcc --help
bash operations/f1tenth/run.sh tuner
bash operations/f1tenth/run.sh localization-test
```

Python 모듈은 실제 분류 경로인 `control.acados_mpcc`, `planning.track_pipeline`, `navigation.lidar_filter` 등을 사용합니다. `run.sh python`은 AAC를 Python 검색 경로에 추가하고 본체의 acados 환경을 사용합니다. 일반 Python으로 직접 실행할 때는 `PYTHONPATH=~/ros2_ws/src/aac_2026`를 설정합니다.

GUI의 기존 `edited_inner` 선택과 제어기 모델·가중치는 유지했습니다. `system/f1tenth/config/selected_run.yaml`은 별도 선택 실험 설정입니다. 3D 위치 추정과 C++ 가상 차량은 실차 연결 완료를 뜻하지 않습니다.

## 새 장비 준비

AAC를 `<workspace>/src/aac_2026`에 받은 뒤:

```bash
python3 operations/f1tenth/setup_workspace.py
bash operations/f1tenth/run.sh check --source-only
```

같은 본체에서 기존 자산·가상환경을 연결할 때만:

```bash
python3 operations/f1tenth/setup_workspace.py \
  --assets /home/irdl4/Workspace/f1tenth_ws --reuse-python
```

이전 데이터 이름이 `bags/`여도 새 워크스페이스에서는 `bag/`로 연결합니다. 기존 파일·비어 있지 않은 폴더와 충돌하면 덮어쓰지 않습니다. AGX에는 본체 가상환경·바이너리를 복사하지 않고 새로 빌드합니다.

현재 본체의 `bag/`, `tracks/`, `results/`, `vendor/acados`, `.venv-*`는 예전 `Workspace/f1tenth_ws`의 원본을 연결합니다. 따라서 예전 워크스페이스를 통째로 삭제하면 안 됩니다. 앞으로 예전 소스를 수정하지 말고 AAC 소스를 수정합니다. 자동 동기화는 하지 않습니다.

## 본체 Jazzy 매핑

```bash
cd ~/ros2_ws
export F1TENTH_WS="$PWD"
docker compose -f src/aac_2026/docker/f1tenth/compose.yaml build fastlio
bash src/aac_2026/navigation/fastlio/scripts/slam.sh build
```

Compose는 `bag/`의 원본을 컨테이너에서도 읽을 수 있게 연결합니다. 빌드 컨텍스트에는 대형 데이터를 보내지 않습니다. 실제 센서 실행에는 네트워크·USB·SDK·차량 TF 설정을 추가로 확인해야 합니다.

## 검증 기록

소스 출처는 `operations/f1tenth/import_manifest.json`, 이번 재분류 내역은 `structure_reuse.json`에 있습니다. 과거 검증·계획 문서는 당시 경로를 포함할 수 있습니다.

이전 C++ 검사 `core_parity`의 차이 `5.34077e-6`은 원본에서도 동일하게 재현된 기존 문제입니다. 이번 정리에서는 수식이나 허용치를 변경하지 않았습니다. AGX 배포·실제 센서·VESC 동작은 별도 검증 대상입니다.
