# F1TENTH 연구 워크스페이스

코드와 설명서는 **[src/f1tenth](src/f1tenth/README.md)**에서 기능별로 찾습니다. 교수님의 `irdl_tutorial`처럼 센서·매핑·계획·제어·시뮬레이션을 나누고, 각 기능 안에 설정·테스트·실행 스크립트·문서를 모았습니다. 원본 데이터와 실험 결과, 빌드 환경은 워크스페이스 루트에 둡니다.

먼저 [현재 실차 완주 계획](src/f1tenth/docs/geonji-full-lap-roadmap.md), [MPCC 가중치 GUI](src/f1tenth/control/mpcc_tuner/docs/mpcc-tuner.md), [현재 CAD·출력 안내](src/f1tenth/cad/README.md)를 확인하세요. 실차 계획의 기준일은 2026-09-30이며, 소스 폴더 정리는 2026-09-22 기준입니다.

## 어디에서 작업하나요?

| 위치 | 내용 |
| --- | --- |
| [sensors](src/f1tenth/sensors/README.md) | bag 입력 어댑터, Livox 메시지 정의 |
| [mapping](src/f1tenth/mapping/README.md) | FAST-LIO·Dynamic-LIO, 이동체 관측점 필터, 지도 검토 |
| [planning](src/f1tenth/planning/README.md) | 주행 영역, 차체 여유, 중심선·경계 추출 |
| [control](src/f1tenth/control/README.md) | acados MPCC, 차량 모델, 가중치 GUI, 실행 결과 분석 |
| [simulation](src/f1tenth/simulation/README.md) | 저장 궤적의 Gazebo 재생 |
| [cad](src/f1tenth/cad/README.md) | 상판 설계, R03 의존 자료, R04·REFIT v2 출력물 |
| [operations](src/f1tenth/operations/README.md) · [docker](src/f1tenth/docker/README.md) | 공통 빌드·유지관리와 실행 환경 |
| [common](src/f1tenth/common/README.md) · [docs](src/f1tenth/docs/README.md) | 공통 경로 처리와 프로젝트 문서 |

```text
f1tenth_ws/
├── src/f1tenth/       기능별 소스·설정·테스트·문서
├── bags/             원본 센서 기록
├── tracks/           생성한 트랙과 지도 자료
├── results/          환경별 실험 결과와 실행 기록
├── build/ install/ log/   ROS·도구 빌드와 로그
├── vendor/           acados 등 외부 의존성
├── .venv-acados/ .venv-dynamic-lio/   기존 Python 환경
└── compose.yaml      워크스페이스 전체를 /ws로 연결하는 설정
```

[결과 목록](results/README.md)은 실내복도·건지광장으로 나뉩니다. 결과 폴더에는 재현에 필요한 설정·생성 solver·검증 근거도 있으므로 소스와 별도로 보존합니다.

## 자주 쓰는 실행

아래 명령은 `f1tenth_ws` 루트에서 필요한 작업을 골라 실행합니다. Python 모듈 이름 `tools.*`와 기존 `scripts/`, `config/` 명령도 계속 사용할 수 있습니다.

```bash
# MPCC 가중치 GUI: http://127.0.0.1:8765
bash src/f1tenth/control/mpcc_tuner/scripts/mpcc_tuner.sh

# 저장 지도 RViz 검토
bash src/f1tenth/mapping/map_review/scripts/rviz_map_review.sh

# 저장된 acados 궤적 Gazebo 재생
bash src/f1tenth/simulation/gazebo_replay/scripts/gazebo_replay.sh
```

[매핑 실행](src/f1tenth/mapping/README.md), [MPCC 명령 실행](src/f1tenth/control/README.md), [Docker 환경](src/f1tenth/docker/README.md)에 작업별 예제가 있습니다. GUI의 기본 코스 `edited_inner`는 이전 지도 기준이며, 현재 계획의 `natural_inner`와 아직 다릅니다. 기준 통일과 실제 차량 제어 연결은 [현재 계획](src/f1tenth/docs/geonji-full-lap-roadmap.md)에 미완료 작업으로 기록합니다.

## 기존 경로와 과거 기록

루트의 `tools/`, `config/`, `scripts/`, `tests/`, `docs/`, `hardware/`, `docker/`, `requirements/`, `patches/`는 기존 명령과 링크를 위한 **호환 경로**입니다. 파일·폴더 링크와 문서 이동 안내로 새 위치에 연결되며, 별도 복제본을 편집하는 구조가 아닙니다. `src/FAST_LIO`, `src/dynamic_lio`, `src/bag_adapter`, `src/livox_ros_driver2`도 새 소스를 가리키는 별칭입니다.

편집기 탐색기에서는 호환 경로를 숨겨 `src/f1tenth` 중심으로 봅니다. 탐색기에서 숨기는 것은 삭제가 아니며, 터미널과 기존 스크립트에서는 계속 접근할 수 있습니다. 새 파일은 해당 기능의 실제 소스 위치에 추가합니다.

[초기 진행 현황](src/f1tenth/docs/history/project-status.md), [초기 연구 로드맵](src/f1tenth/docs/history/project-roadmap.md), [정리 전 워크스페이스 안내](src/f1tenth/docs/history/workspace-before-organization.md)는 당시 사실과 재현 명령을 보존한 과거 기록입니다.

[폴더 정리 결과·검증·복구 안내](src/f1tenth/operations/maintenance/README_KO.md)
