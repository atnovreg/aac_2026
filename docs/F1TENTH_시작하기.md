# F1TENTH 작업 위치와 실행

**앞으로 소스는 `ros2_ws/src/aac_2026`에서 편집합니다.** 본체와 Orin Nano에서 사용하던 코드를 AAC 분류에 통합했습니다. 기존 AAC 노드는 유지했습니다. 기준일은 2026-11-05입니다.

## 어디에 무엇이 있나요?

| 위치 | 용도 |
|---|---|
| `sensors/livox_ros_driver2` | Nano에서 가져온 실제 MID-360 드라이버. 본체의 메시지 전용 패키지를 대체 |
| `sensors/livox_front_filter` | 센서 좌표계 전방 필터·RViz. 위치 추정기는 아님 |
| `sensors/realsense-ros`, `sensors/realsense_capture` | D435i 드라이버와 촬영 도구 |
| `sensors/bag_adapter` | 과거 PointCloud2 bag 입력 어댑터 |
| `navigation/fastlio` | ROS2 FAST-LIO와 기존 bag 매핑 도구 |
| `navigation/localization3d` | 최근 3D ICP·wheel/gyro·MPCC 오프라인 실험과 테스트 |
| `navigation/lidar_filter`, `navigation/map_review` | PC에서 사용하는 지도 처리·검토 |
| `planning/geonji`, `planning/track_pipeline` | 지도 기반 경로·차체 여유 처리 |
| `control/acados_mpcc`, `control/mpcc_cpp_virtual` | 기존 MPCC 모델·solver 및 C++ 가상 차량 노드 |
| `control/mpcc_tuner`, `control/run_analysis` | PC용 가중치 GUI·결과 분석 |
| `simulation/f1tenth_gazebo_replay` | PC용 기록 궤적 Gazebo 재생 |
| `system/f1tenth` | PC·Nano·AGX 역할, 선택한 경로·제어 설정 |
| `docker/f1tenth` | F1TENTH의 Jazzy PC 환경과 Nano에서 보존한 Livox SDK 소스 |
| `archive/f1tenth` | 이전 워크스페이스·CAD·ROS1 매핑·별도 실험 보관 위치 |

기존 `pure_pursuit_node`, `stanley_node`, `joy_control_node` 등은 같은 AAC 안에서 계속 찾을 수 있습니다. MPCC와 이 노드들의 실제 입출력·명령 선택 연결은 별도 통합 작업입니다.

## 본체와 Jetson을 어떻게 구분하나요?

소스의 기준은 AAC 하나입니다. **PC는 지도 생성·시뮬레이션·튜닝·분석**, **Jetson은 센서·추정·제어 실행**을 담당합니다. 공통 알고리즘은 복제해서 따로 수정하지 않습니다. 장비별 설정과 빌드 산출물만 구분합니다.

- 현재 Nano: Orin Nano Super, Ubuntu 24.04.5, L4T 39.2.1, 기존 `f1tenth` 컨테이너의 ROS2 Jazzy를 확인했습니다. 설치된 JetPack 메타 패키지 버전은 확인되지 않았습니다.
- AGX: 앞으로 배포할 대상입니다. 실제 OS·JetPack·CUDA·드라이버를 확인한 뒤 소스와 선택 데이터를 옮겨 새로 빌드합니다. Nano 컨테이너·가상환경·바이너리를 그대로 복사하지 않습니다.
- 기존 AAC PC 설정은 Humble입니다. 가져온 F1TENTH는 Jazzy 환경을 사용합니다. **서로의 `install/setup.bash`를 섞어 source하지 않습니다.**
- 새 F1TENTH ROS 산출물은 `ros2_ws/artifacts/f1tenth/jazzy-x86_64/` 또는 `jazzy-aarch64/` 아래의 `build/install/log`에 둡니다. 기존 AAC의 루트 `build/install/log`는 유지합니다.

현재 Nano에는 AAC를 배포하거나 기존 실행을 변경하지 않았습니다. 센서·MPCC 소스의 보존과 파일 정리가 실차 자율주행 완료를 뜻하지 않습니다.

### 지금 어떤 장비에서 작업하나요?

**코드는 본체에서 작성하고, 실제 장치와 연결해 실행·확인하는 작업은 Jetson에서 합니다.** 같은 AAC 소스를 사용합니다.

| 작업 | 할 곳 |
|---|---|
| 센서→위치 추정→MPCC→VESC 연결 코드 작성 | 본체 |
| 녹화 데이터·가상 차량·가상 출력으로 연결 검사 | 본체 |
| Jetson용 ROS 패키지·SDK·acados·solver 빌드 | 해당 Jetson |
| 실제 LiDAR·IMU·VESC 수신/송신 연결과 실행 확인 | 해당 Jetson |
| 실제 차량에 실어 자율주행 프로그램 실행 | 차량의 Jetson |
| 저장된 로그 분석, 지도·경로·제어기 수정 | 본체 |

권장 흐름은 **본체에서 개발·가상 검사 → Jetson에 같은 소스 배포·빌드 → 실제 장치 확인 → 로그를 본체에서 분석**입니다. AGX를 기다리는 동안 현재 Nano에서 확인 가능한 장치 연결을 시험할 수 있습니다. AGX가 준비되면 그 환경에서 다시 빌드하고 검증합니다. 이번 정리에서는 Nano의 기존 환경을 수정하거나 실제 장치 출력을 실행하지 않았습니다.

본체 경로는 `/home/irdl4/ros2_ws/src/aac_2026`입니다. Jetson의 사용자 이름·설치 경로가 본체와 완전히 같을 필요는 없습니다.

## 지금 바로 확인할 명령

```bash
cd /home/irdl4/ros2_ws/src/aac_2026
bash f1tenth.sh check
bash f1tenth.sh python -m tools.acados_mpcc --help
bash f1tenth.sh localization-test

# PC의 오프라인 가중치 GUI
bash f1tenth.sh tuner
```

GUI는 기존 `edited_inner` 설정을 유지합니다. `system/f1tenth/config/selected_run.yaml`의 `natural_inner`와 동일한 기본 코스로 바꾼 것은 아닙니다. 이번 작업은 폴더·환경 정리이며 제어기 설정·모델·주행 동작은 유지합니다.

3D 위치 추정 코드는 실제 MID-360 실시간 추정 노드가 아닌 오프라인 실험입니다. 새 결과는 `navigation/localization3d/results`에 저장합니다. 과거 검증 결과는 `archive/f1tenth/localization_results`에 보존되어 있습니다. 실험의 `baseline/`과 두 solver 어댑터도 함께 가져왔으므로 이전 임시 소스 폴더를 import할 필요가 없습니다.

## 큰 데이터와 기존 Python 환경

본체의 `ros2_ws/bags`, `tracks`, `results`, `vendor/acados`, `.venv-*`는 기존 `f1tenth_ws`의 자산을 가리키는 **로컬 연결**입니다. 데이터나 가상환경을 중복 복사하지 않았습니다. `tools/`, `config/`, `scripts/`는 AAC 내부 소스를 가리키는 호환 별칭입니다.

**아직 기존 `f1tenth_ws`와 별도 실험 원본을 삭제하면 안 됩니다.** 데이터 연결과 현재 실행 중인 기존 화면이 그 위치를 사용합니다. 정리 전 원본은 복구와 과거 실행 재현용이며, 앞으로의 코드 편집은 AAC에서 합니다. 예전 폴더의 코드를 AAC로 자동 동기화하지 않습니다.

새 본체나 AGX에 AAC를 `ros2_ws/src/aac_2026`으로 가져온 뒤에는 다음으로 소스 별칭을 만듭니다.

```bash
python3 operations/f1tenth/setup_workspace.py

# 같은 본체의 기존 자산·Python 환경을 연결하는 경우에만 사용
python3 operations/f1tenth/setup_workspace.py \
  --assets /home/irdl4/Workspace/f1tenth_ws --reuse-python
```

`--reuse-python`은 같은 본체에서만 사용합니다. AGX에는 Python 환경과 acados를 새로 설치합니다. 별칭 생성은 기존 파일과 충돌하면 덮어쓰지 않고 중단합니다. 소스 보존 목록은 `operations/f1tenth/import_manifest.json`에 있습니다.

## Jazzy PC 컨테이너

```bash
cd /home/irdl4/ros2_ws
export F1TENTH_WS="$PWD"
export F1TENTH_ASSETS=/home/irdl4/Workspace/f1tenth_ws
docker compose -f src/aac_2026/docker/f1tenth/compose.yaml build fastlio
docker compose -f src/aac_2026/docker/f1tenth/compose.yaml run --rm fastlio \
  bash scripts/build.sh
```

PC 이미지 이름은 `aac-f1tenth-*`로 기존 이미지와 구분합니다. `track`은 `fastlio`, `gazebo`는 `track` 이미지 빌드 후 만듭니다. 컨테이너는 로컬 데이터 링크를 해석할 수 있도록 기존 bags/tracks/results/acados 경로를 별도로 연결합니다. Docker 빌드 컨텍스트에는 대형 데이터가 포함되지 않습니다.

실제 센서 실행에는 장비 네트워크·USB·SDK 설정이 추가로 필요합니다. Nano의 Livox SDK 커밋과 수정된 두 헤더를 보존했으며 PC Dockerfile은 이 소스에서 SDK를 빌드합니다. RealSense의 `librealsense`/RSUSB도 AGX에서 별도로 빌드해야 합니다. 기존 AAC `start.sh`는 기존 AAC 스택을 실행하며, 이번 F1TENTH를 자동으로 시작하도록 변경하지 않았습니다.

전방 필터 호스트 스크립트는 준비된 AAC 컨테이너 이름을 `AAC_CONTAINER`로 명시해야 합니다. 기존 Nano의 `f1tenth` 컨테이너를 자동으로 대상으로 선택하지 않습니다.

## AGX로 가져갈 때

1. AAC 소스와 `system/f1tenth/config`를 가져옵니다.
2. 필요한 지도·경로·설정만 `bags/tracks/results`의 같은 상대 경로로 준비합니다. 원본 전체 bag과 과거 빌드 결과는 주행에 필요하지 않습니다.
3. 로컬 연결을 AGX의 실제 데이터 위치에 맞게 재구성합니다. 본체의 `.venv-*`, `build/install/log`, `.so`, SDK 설치물을 복사하지 않습니다.
4. SDK·acados·ROS 패키지·solver를 AGX에서 빌드하고 센서 수신과 가상 출력을 먼저 검증합니다.

실제 AGX 배포와 VESC 출력 연결은 아직 수행하지 않았습니다. 파일 정리 검증 범위는 `operations/f1tenth/verification.json`에 기록합니다.

## 이번 정리에서 확인한 범위

- Python·정리·별칭·wrapper 검사 84개 통과.
- Livox 실제 드라이버·bag 어댑터·FAST-LIO의 Jazzy x86_64 빌드 통과.
- 전방 필터의 메시지 보존 검사와 Gazebo 궤적 재생 plugin 빌드·보간 검사 통과.
- 가져온 파일 901개, 기존 AAC 원본 548개와 Nano 원본 585개의 보존 확인.
- C++ native 실행 파일 빌드는 통과했고 검사는 9/10 통과했습니다. `core_parity`의 명령 차이는 `5.34077e-6`으로 기존 허용치 `1e-6`을 넘었습니다. 같은 새 artifact로 정리 전 원본 소스를 빌드해도 정확히 동일하게 재현됐습니다. 이번 작업에서는 제어기 수식·허용치를 변경하지 않고 기존 문제로 기록했습니다.

파일·빌드 검증 결과와 실제 센서·실차·AGX 실행 검증을 구분합니다.
