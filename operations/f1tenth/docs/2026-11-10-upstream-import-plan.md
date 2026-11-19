# AAC 원본 변경 선별 반영 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 현재 MPCC·FAST-LIO 구성을 유지하면서 본체 시뮬레이션과 실차의 시간 설정을 구분한다. 원본의 도로 인식·지도·영상 위치 보정은 현재 목표에 쓸 용도가 확인된 기능만 별도 검증한다.

**Architecture:** 원본 브랜치 전체를 병합하지 않고 기능별 변경을 고정된 커밋에서 선별한다. 기존 패키지는 필요한 변경만 적용하고, 소스·설정·launch·설치 목록·사용 문서를 함께 반영한다. 각 반영 커밋에 원본 저장소와 커밋을 기록한다.

**Tech Stack:** ROS2 Humble의 기존 AAC, ROS2 Jazzy의 F1TENTH, Python/OpenCV, C++/ament/colcon, Gazebo.

**Spec:** [이 문서의 범위와 도입 결정](2026-11-10-upstream-import-plan.md#범위와-도입-결정). 사용자에게 제안하는 계획이며 구현을 시작한 상태가 아니다.

## 범위와 도입 결정

현재 구성을 이어서 개발한다는 전제로 정했다. 목적은 원본 저장소의 모든 실험을 복제하는 것이 아니라 현재 개발에 쓸 수 있는 기능을 추가하는 것이다.

사용자 확인: 원본의 주차·미션 전체 구성은 다른 대회를 위한 코드이며 이번 목표와 관계없다. 이번 도입 범위와 후속 후보에서 제외한다.

이번 수정안에서는 MPPI 제어기와 IPM costmap 실험도 도입 대상에서 제외한다. HSV·IPM 지도는 현재 MPCC에 연결되는 기능으로 확인된 것이 아니므로, 필수 도입 목록에서 별도 검증 후보로 옮긴다.

검토한 방식:

1. `main`의 4커밋 전체 병합: 이력 연결은 간단하지만 MPPI 기본 제어 방식까지 바뀐다.
2. 기능별 선별 반영: 출처를 따로 기록해야 하지만 현재 제어기·환경을 유지하면서 검증할 수 있다. **추천안이며 이 계획의 기준이다.**
3. `sim`·주행 데이터 브랜치까지 전체 병합: 서로 다른 실험과 장비 설정이 섞이므로 이번 범위에서는 선택하지 않는다.

### 비교 기준

2026-11-10 원격 조회와 로컬 코드 비교 결과:

| 대상 | 고정 커밋 | 현재 HEAD에 없는 커밋 | 판단 |
|---|---|---:|---|
| 현재 작업 | `3bd9a46299fd2e3c888f08b3c4cb6cbc20d00e0f` | — | 보호할 기준 |
| upstream/main | `5be1d8a93e91475bdba7259dfda95e9eb98605a2` | 4 | 현재 실행 구성의 시간 설정을 선별. HSV·IPM은 별도 검증 후보 |
| upstream/sim | `08e085067f0c4f7edf7ee68ad19a0f38e66ed273` | 5 | 트랙·LiDAR·영상 기능을 개별 판단 |
| upstream/data/latest5-yolop-minima-drives-20260727 | `33835b6021125e50ae02e85ebd367913614bb275` | 91 | 영상 인식과 위치 보정의 후속 검토 자료 |

`main`은 `git merge-tree --write-tree --name-only HEAD upstream/main`에서 텍스트 충돌이 없었다. 그러나 MPPI의 기본값이 경로 비용 0, costmap 필수인 실험 설정으로 바뀌므로 자동 병합 가능 여부를 채택 기준으로 삼지 않는다.

## Global Constraints

- 기존 AAC는 ROS2 Humble, F1TENTH는 ROS2 Jazzy이며 build/install 환경을 분리한다.
- 사용자가 정한 실행 기준: 본체의 Gazebo 시뮬레이션은 `use_sim_time=true`, 실차는 `use_sim_time=false`로 실행한다. 각 구성에서 함께 동작하는 센서·위치 추정·계획·제어 노드에 같은 시간 설정을 전달한다.
- 기존 MPCC·FAST-LIO, Frenet의 LaserScan 장애물 회피, 장비 보정값과 하드웨어 출력 설정을 유지한다.
- 기존 `sensors/perception/navigation/planning/control/system/docker/operations` 분류를 사용한다.
- `bag`, `tracks`, `results`, 가상환경과 vendor 연결의 기존 대상을 보존한다.
- `navigation/WildGS-SLAM`의 untracked 내용과 `perception/BiSeNet`의 로컬 변경을 보존한다.
- 서브모듈을 원격 최신 버전으로 일괄 갱신하지 않는다.
- 새 기능은 명시적으로 실행해 검증하며 기본 주행 시작 구성에 자동으로 추가하지 않는다.
- 실차 실행·AGX 배포는 이 반영 작업의 완료 조건에 포함하지 않는다.
- 다른 대회용 주차·미션 파이프라인 및 그 전용 메시지·launch·제어 설정을 이번 목표를 위한 코드로 가져오지 않는다. 기존 로컬 코드를 삭제하는 작업은 포함하지 않는다.
- MPPI 코드·기본 설정·launch 변경을 도입하지 않는다. HSV·IPM 출력을 MPCC 입력으로 자동 연결하지 않는다.

## Review Focus

- 시간: `/clock` 사용 여부와 노드의 `use_sim_time`이 일치해야 한다. 실시간 모드에서 `/clock`이 없어도 입력을 처리해야 한다.
- 주행 가능 영역: 도로·주차장·잔디를 구분하고, 허용 옵션을 켜도 잔디를 주행 가능으로 만들지 않아야 한다.
- 좌표: IPM 격자의 원점·축 방향·frame과 실제 카메라 TF가 일치해야 한다. Gazebo용 기본 외부 보정을 실차 값으로 간주하지 않는다.
- 입력 중단: IPM을 검증하는 경우 영상·CameraInfo·TF가 누락될 때 새 격자의 생성 여부와 timestamp 동작을 확인한다. 결과를 주행 제어 입력으로 연결하는 작업은 이번 범위에 없다.
- 회귀: 기존 경로·제어·지도 파일이 바뀌지 않아야 하며, 새 기능을 실행하지 않는 기존 구성도 동작해야 한다.

## 1차 반영: 현재 실행 구성 확인 → 시간 설정

### Task 1: 기준과 검증 환경 준비

**Files:** 현재 Git 상태, `operations/f1tenth/run.sh`, `operations/f1tenth/docs/F1TENTH_시작하기.md`, `operations/f1tenth/docs/AAC_HUMBLE.md`와 실제 사용하는 launch·실행 스크립트. 제품 코드 수정 없음.

**Interfaces:** 현재 HEAD와 기존 센서 입력을 기준으로 뒤 작업의 비교 결과를 만든다.

- [ ] 실행 시 작업 상태를 다시 확인하고 사용자 변경이 없는 격리된 작업 공간에서 진행한다. 고정 기준 이후 변경이 있으면 덮어쓰지 말고 차이를 재평가한다.
- [ ] 기존 `operations/f1tenth/run.sh check --source-only` 결과와 원래 AAC 빌드 대상 목록을 기록한다.
- [ ] 본체 시뮬레이션과 실차에서 실제 사용할 노드·실행 입구·ROS 배포판·입출력 토픽을 확인한다. 다른 대회용 노드나 MPPI를 시간 설정 정리 명목으로 추가하지 않는다.
- [ ] 해당 환경에서 시간 설정을 확인할 짧은 bag 또는 Gazebo 입력을 선택한다. HSV·IPM도 검증할 경우 카메라·CameraInfo·TF가 있는 입력을 별도로 준비한다. 소스 검사와 실행 검증 결과를 구분해 보고한다.
- [ ] 후속 기능별 커밋 본문에 원본 저장소, SHA, 가져온 범위와 의도적으로 제외한 설정을 기록한다.

### Task 2: `use_sim_time` 전달 정리

**Source:** `d6b01bc`의 시간 설정 전달 방식을 참고하되 현재 실행 구성에 필요한 변경만 적용한다.

**Files:** Task 1에서 확인한 실행 입구와 실제 사용하는 노드의 launch·설정만 수정한다. 기존 AAC Docker를 사용하는 경우 `docker/{env.sh,run_x86.sh}`와 해당 tmux 설정의 시간 전달을 확인한다. 원본의 `planning/*/launch/`나 제어 패키지를 일괄 반영하지 않는다.

**Interfaces:** launch 인수 `use_sim_time`과 환경변수 `USE_SIM_TIME`을 노드 파라미터로 전달한다. MPPI 알고리즘·제어 가중치와 Docker의 현재 bag 마운트는 가져오기 대상이 아니다.

- [ ] 현재 실행 경로와 원본 변경의 교집합을 확정한 뒤 시간 설정 변경만 적용한다. 새 HSV/IPM launch 변경은 해당 후보를 검증하기로 했을 때 Task 3·4에서 함께 반영한다.
- [ ] 본체의 Gazebo 실행 구성은 `USE_SIM_TIME=true`, 실차 실행 구성은 `USE_SIM_TIME=false`를 명시하고 각 launch의 `use_sim_time`으로 전달한다. 개별 노드의 기본값에 의존하지 않도록 실행 입구와 사용 안내를 함께 정리한다.
- [ ] bag 재생은 `/clock` 발행 여부에 맞춰 별도 설정한다. 실제 센서로 동작하는 실차 구성에는 시뮬레이션 시간 기본값이 전파되지 않는지 확인한다.
- [ ] 실행 중 `ros2 param get /<해당 노드> use_sim_time`으로 인수 전달을 확인한다. 시뮬레이션 일시정지·재개와 실시간 모드의 입력 처리를 각각 확인한다.
- [ ] 수정한 shell 스크립트의 `bash -n`과 영향받은 패키지의 해당 ROS 배포판 빌드를 통과시킨 후 시간 설정만 별도 커밋한다. 첫 반영의 완료는 Task 1~2까지다.

## 별도 검증 후보: HSV 도로 인식·IPM 지도

Task 3~4는 자동 착수할 도입 작업이 아니다. 현재 시뮬레이션·분석에서 사용할 목적을 확인한 경우에만 별도로 진행한다. 원본의 색상 기준은 트랙 텍스처용이며 현재 실차 도로에서도 유효하다고 가정하지 않는다. 이번 검증에 MPCC 입력 변경은 포함하지 않는다.

### Task 3: HSV 도로 인식 추가

**Source:** `cbc88bc`의 HSV 기능과 `5be1d8a` 시점의 관련 시간 설정.

**Files:** `perception/road_seg_node/road_seg_node/{hsv_drivable_core.py,hsv_drivable_node.py}`, `config/hsv_drivable_node.yaml`, `launch/hsv_drivable_node.launch.py`, `setup.py`, `README.md`. 새 검증: `perception/road_seg_node/test/test_hsv_drivable_core.py`.

**Interfaces:** `/ipm_bev/front`의 BGR 영상 → `/drivable_hsv/`의 도로·주차장·잔디 마스크와 class map. 기존 YOLO 모델과 노드를 대체하지 않는다.

- [ ] 위 기능의 코드·설정·설치 목록을 함께 가져오고, 기존 `road_seg_node` 실행 경로는 유지한다.
- [ ] `segment_hsv_drivable(image_bgr, HsvDrivableConfig)`에 합성 도로·주차장·잔디 패치를 입력하는 테스트를 추가한다. 클래스 값 1·2·0, ROI 바깥 0, 형태학 처리 뒤 잔디 제외를 확인한다.
- [ ] `PYTHONPATH=perception/road_seg_node python3 -m pytest perception/road_seg_node/test/test_hsv_drivable_core.py -q`가 통과해야 한다. 노드에서는 `allow_parking_area` 전후 주차장 마스크만 바뀌는지 확인한다.
- [ ] Humble에서 패키지를 빌드하고 선택한 입력의 마스크·출력 크기·처리율을 기록한 뒤 별도 커밋한다. 실차 조명에서의 정확도를 주장하지 않는다.

### Task 4: IPM 확률 지도 추가

**Source:** `cbc88bc`의 확률 지도 기능과 `5be1d8a` 시점의 관련 시간 설정.

**Files:** `perception/ipm_bev_node/{CMakeLists.txt,package.xml,README.md}`, `config/ipm_probability_grid.yaml`, `launch/probability_grid.launch.py`, `include/ipm_bev_node/ipm_probability_grid_node.hpp`, `src/{ipm_probability_grid_main.cpp,ipm_probability_grid_node.cpp}`.

**Interfaces:** 이 노드는 raw 전방 카메라·CameraInfo·TF를 직접 사용하며 Task 3의 HSV 노드 출력을 구독하는 구성은 아니다. `/ipm_probability_grid`에 `nav_msgs/OccupancyGrid`를 발행한다. 알려진 값은 0~100, 관측 불가 셀은 -1이다.

- [ ] 기존 BEV 노드에 확률 지도 실행 파일을 추가하고 현재 BEV 실행 동작은 유지한다.
- [ ] `colcon build --packages-select ipm_bev_node road_seg_node`를 Humble 전용 작업 공간에서 통과시킨다.
- [ ] 입력 재생 후 `ros2 topic echo /ipm_probability_grid --once`와 `ros2 topic hz /ipm_probability_grid`로 frame·해상도·원점·데이터 범위·출력 주기를 확인한다. RViz에서 카메라 투영과 지면 방향을 확인한다.
- [ ] 입력 누락 시나리오: 영상 중단, CameraInfo 누락, TF 누락을 각각 재현한다. 유효 격자가 새로 생성되는지와 timestamp 동작을 기록하고, 잘못된 좌표로 새 격자를 발행하는 경우 반영 전에 수정·재검증한다.
- [ ] 도로는 낮은 비용, 잔디는 높은 비용, 관측 밖은 -1인지 확인한다. 주차 허용 옵션을 바꾸면 주차장 비용만 낮아지는지 확인한다.
- [ ] 기존 실행 구성과 F1TENTH 소스 배치 검사를 다시 확인한 뒤 별도 커밋한다. 결과물은 영상·지도 확인용이며 기본 주행 구성에는 연결하지 않는다.

## 필요성이 확인된 뒤 검토: 영상 차선·위치 보정

현재 FAST-LIO의 구체적인 부족점이 확인되고 영상 보정으로 개선할 목적이 정해졌을 때만 검토한다. `33835b6` 브랜치는 91커밋에 실험·되돌림·장비 설정이 섞여 있으므로 아래 순서로 별도 계획을 확정한다.

1. `perception/yolop_node`의 차선 인식·confidence·geometry 코드와 관련 테스트를 도입 단위로 묶는다. 최종 실행 구성이 `seg_lane_node`를 사용하므로 YOLOPv2와 학습된 lane segmentation 모델 중 하나를 입력 데이터로 비교한 뒤 선택한다.
2. 해당 모델의 실제 weight 확보와 오프라인 실행을 확인한다. 코드가 있다는 이유로 모델까지 준비됐다고 간주하지 않는다.
3. `navigation/{lane_map_projector,lane_pose_correction,lane_fusion_node}`는 입력 frame·pose·covariance 계약과 제공 테스트를 확인한다. 먼저 bag에서 기존 FAST-LIO 결과와 비교하고 위치 추정 기본 출력을 교체하지 않는다.
4. `resilient_navigation_node`는 main의 초기 구현을 먼저 넣고 다시 교체하지 않는다. 후속 브랜치의 geometry 테스트와 보정 입력 검증까지 포함해 비교할 때 도입 여부를 정한다.

## 이번에 보류할 항목

| 항목 | 근거 / 도입 조건 |
|---|---|
| `sim` 트랙·울타리 | 12m→11m로 좌표 범위가 바뀌고 울타리가 주차장 진입도 막는다. 별도 실험 월드와 지도 좌표를 함께 검증할 때 도입한다. |
| `lidar_odometry_node`·`fast_gicp`·`ndt_omp` | 2D LaserScan 등록 기반의 별도 위치 추정 계열이다. 현재 FAST-LIO 비교 실험이 필요할 때만 의존성과 함께 가져온다. |
| DUSt3R·추가 visual odometry | 추가 서브모듈·모델·런타임 비용이 있다. 현재 위치 추정의 구체적 부족점이 확인되면 검토한다. |
| 영상 압축 해제 노드 | 사용할 bag과 카메라의 compressed/raw 인터페이스가 요구할 때만 추가한다. |
| 원본 실차 보정·조향·속도·직렬 포트 설정 | 현재 차량·AGX와 동일하다는 근거가 없다. 알고리즘과 분리해 해당 장비에서 검증한다. |
| 주행 기록·모델 일괄 복사, 강의 PPT·화면 결과물 | 필요한 검증 입력·모델만 고른다. 대형 자산은 실제 파일/LFS 확보 여부를 별도 확인하고 bag은 워크스페이스 `bag/`에서 관리한다. |

## 이번 목표에서 제외

- MPPI 및 IPM costmap 기반 제어 실험: 현재 MPCC 중심의 작업에 필요한 기능으로 확인되지 않았다. 도입 후보에서 제외한다.
- 주차·미션 전체 파이프라인: 사용자가 다른 대회용 코드임을 확인했다. 보류 중인 도입 후보로 관리하지 않는다.
- 공통 패키지의 기능을 선별할 때도 주차·미션 전용 의존성이 딸려오지 않도록 변경 범위를 확인한다.

## 완료 보고에 포함할 것

- 가져온 기능과 출처 SHA, 변경한 파일, 유지한 기본 실행 방식.
- 실행한 빌드·테스트와 입력 자료, 출력 frame·시간 설정·처리율.
- 준비되지 않아 수행하지 못한 실행 검증과 다음 단계 착수 조건.
- 기능별 커밋으로 되돌릴 수 있게 하고 검증되지 않은 기능을 기본 주행 구성에 연결하지 않는다.

## 2026-11-10 실행 결과

- 현재 실행 경로를 확인했다. MPCC·localization3d는 오프라인 계산이며 Gazebo 궤적 재생은 자체 시뮬레이션 시간을 사용한다. 실시간 ROS MPCC·실차 통합 실행기는 이번에 추가하지 않았다.
- FAST-LIO launch와 RViz에 `명시적 use_sim_time 인수 > USE_SIM_TIME 환경변수 > false` 순서로 같은 설정을 전달하도록 수정했다. Compose `fastlio` 서비스도 환경변수를 전달한다. 본체의 ROS 시뮬레이션 입력에는 true를 명시한다.
- 실제 MID-360 녹화용 front180 launch는 드라이버·필터·표시 노드를 false로 실행하도록 명시했다. bag 매핑의 기존 true 설정은 유지했다.
- 다른 대회용 기존 AAC Humble 실행기, MPPI, HSV·IPM 및 영상 위치 보정 코드는 가져오거나 변경하지 않았다.
- 검증: 실제 FAST-LIO·RViz의 시간 파라미터 5개 조합 통과, `/clock` 정지 상태의 기존 bag 매핑 107개 pose 처리 통과, 작업 관리 회귀 검사 22개 통과. Jazzy `fast_lio` 빌드·설치도 갱신했다.
- 실차 센서 연결과 Gazebo 센서 시뮬레이션의 전체 주행 검증은 수행하지 않았다. 설치된 FAST-LIO는 기존 C++ 코드이며 이번 변경은 launch·실행 설정이다.
