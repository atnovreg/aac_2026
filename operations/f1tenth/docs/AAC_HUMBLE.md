# 기존 AAC Humble 실행

기존 AAC 노드를 사용하는 환경입니다. 추가한 F1TENTH Jazzy 환경은 [시작 안내](F1TENTH_시작하기.md)를 따릅니다. 두 환경의 설치 결과를 섞어 source하지 않습니다.

소스를 받는 명령은 저장소 루트 README의 `atnovreg/aac_2026` 안내를 사용합니다. 이미 받은 소스에서 실행합니다.

```bash
cd ~/ros2_ws/src/aac_2026/docker

# 사용할 플랫폼 하나를 선택
bash build_docker.sh x86
bash run_x86.sh
```

플랫폼별 기존 진입점:

| 플랫폼 | 이미지 빌드 인수 | 컨테이너 실행 |
|---|---|---|
| 본체 x86 | `x86` | `run_x86.sh` |
| Mac | `mac` | `run_mac.sh` |
| Jetson | `ros2` | `run_jetson.sh` |

이 표의 Jetson 스크립트가 현재 AGX 환경에서 검증됐다는 의미는 아닙니다. 실제 OS·JetPack·드라이버를 먼저 확인합니다.

컨테이너 안에서:

```bash
cd /ros_ws/src/aac_2026
bash build.sh
source /ros_ws/install/setup.bash
```

실행 구성은 기존 `start.sh`, `auto.sh`, `operations/tmuxp_config/`를 확인합니다. 이 스크립트들은 여러 노드를 실행하므로 사용하려는 차량·시뮬레이션 구성을 확인한 뒤 사용합니다. 추가한 MPCC·FAST-LIO 전체가 자동 연결되는 구성은 아닙니다.

Docker·GPU·USB·DDS 준비는 플랫폼별 `docker/run_*.sh`, `docker/dockerfiles/`, `operations/pc_setup/`의 실제 설정을 확인합니다. 기존 설치 문서와 과거 결과는 Git 커밋 `5faa7e0` 및 본체의 `log/f1tenth/migration-20261105/`에 보존했습니다.
