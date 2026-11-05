# Docker 실행 환경

| 파일 | 용도 |
| --- | --- |
| [Dockerfile](Dockerfile) · [entrypoint.sh](entrypoint.sh) | ROS 2 Jazzy·FAST-LIO 실행 |
| [Dockerfile.track](Dockerfile.track) | 지도 기반 트랙 분석 |
| [Dockerfile.acados](Dockerfile.acados) | acados MPCC 환경 |
| [Dockerfile.dynamic_lio](Dockerfile.dynamic_lio) | ROS 1 Noetic·Dynamic-LIO 환경 |
| [Dockerfile.gazebo](Dockerfile.gazebo) · [gazebo-entrypoint.sh](gazebo-entrypoint.sh) | Gazebo 재생 환경 |

[compose.yaml](../../../compose.yaml)은 워크스페이스 루트에 있습니다. Docker의 `/ws`에 워크스페이스 전체를 연결하므로 원본 `bags/`, 결과·빌드 경로와 가상환경은 소스 묶음 밖의 기존 위치를 사용합니다. Dynamic-LIO는 [전용 실행 스크립트](../mapping/dynamic_lio/scripts/dynamic_lio.sh)를 사용합니다.

이미지·실행 작업은 각 기능의 안내를 따릅니다: [mapping](../mapping/README.md), [control](../control/README.md), [simulation](../simulation/README.md). 루트의 `docker/`는 이 폴더를 가리키는 호환 경로입니다.

[소스 안내](../README.md)
