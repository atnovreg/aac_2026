# AAC F1TENTH Docker 환경

기존 `docker/` 분류 안에서 F1TENTH의 Jazzy 환경을 관리합니다. 기존 AAC Humble 환경과 설치 경로를 섞지 않습니다.

실행 구성은 이 폴더의 [compose.yaml](compose.yaml)입니다. 워크스페이스 루트에 별도 Compose 파일을 만들지 않습니다.

```bash
cd ~/ros2_ws
docker compose -f src/aac_2026/docker/f1tenth/compose.yaml build fastlio
bash src/aac_2026/navigation/fastlio/scripts/slam.sh shell
```

`fastlio`는 매핑, `track`은 경로 처리, `acados`는 MPCC, `gazebo`는 기록 궤적 시각화 환경입니다. 컨테이너의 Python 검색 경로는 `/ws/src/aac_2026`이며 실제 AAC 분류의 모듈을 불러옵니다. 이미지 기본 빌드 순서는 `fastlio` → `track` → `gazebo`입니다.

`vendor/Livox-SDK2`에는 Nano에서 사용한 SDK 소스와 수정 헤더를 보존했습니다. `context.dockerignore`는 큰 데이터와 빌드 결과가 Docker 빌드에 들어가지 않게 합니다. 워크스페이스의 숨김 `.dockerignore`는 이 파일을 연결합니다.

Compose는 워크스페이스의 `bag/`, `tracks/`, `results/`를 같은 컨테이너 경로에 직접 연결합니다. 본체에서 심볼릭 링크로 쓰거나 새 장비에서 실제 폴더로 써도 같은 구조입니다. 없는 데이터 폴더를 Docker가 임의로 생성하지 않도록 설정했습니다. 이 구성은 실제 AGX의 센서·USB·GPU 환경을 검증한 배포 완료 구성이 아닙니다.
