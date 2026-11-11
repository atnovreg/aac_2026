# AAC 자율주행 개발

**PS5 수동 제어 / Jetson AGX Orin:** [젯슨 설치·실행 안내](system/f1tenth_manual/JETSON_QUICKSTART.md).
이 작업 브랜치에는 PC에서 검증한 입력·정지·USB 재연결 코드와 젯슨용 선택 빌드가 포함돼 있습니다.
조향 및 지상 주행의 하드웨어 검증 범위는 해당 안내를 확인하세요.

앞으로 수정할 코드는 `~/ros2_ws/src/aac_2026`에 있습니다. 본체와 Orin Nano의 F1TENTH 코드를 기존 AAC의 센서·위치 추정·계획·제어 분류에 통합했습니다.

| 기존 분류 | 추가한 작업 |
|---|---|
| `sensors/` | MID-360·RealSense 드라이버, 전방 필터, bag 어댑터 |
| `navigation/` | FAST-LIO, 지도 필터·검토, 오프라인 3D 위치 추정 |
| `planning/` | 건지광장 경로 작업, track 생성 |
| `control/` | MPCC, C++ 가상 차량, 튜너·분석 |
| `system/` | F1TENTH 장비 설정·공통 경로 함수, Gazebo 궤적 재생 |
| `docker/` | F1TENTH Docker 환경·Livox SDK 소스 |
| `operations/` | 실행·설치 보조, 검증, 설명서·과거 작업 색인 |

[작업 위치·실행 방법](operations/f1tenth/docs/F1TENTH_시작하기.md) · [장비 설정·AGX 자료 목록](system/f1tenth/README_KO.md) · [과거 작업 보관 위치](operations/f1tenth/archive/README_KO.md)

## 새 장비에 소스 받기

비공개 저장소는 `atnovreg/aac_2026`이며 원본 `Leedk3/aac_2026`의 커밋 이력과 서브모듈 출처를 유지합니다.

```bash
gh auth login
mkdir -p ~/ros2_ws/src
gh repo clone atnovreg/aac_2026 ~/ros2_ws/src/aac_2026 -- --recurse-submodules
cd ~/ros2_ws/src/aac_2026
python3 operations/f1tenth/setup_workspace.py
bash operations/f1tenth/run.sh check --source-only
```

설치는 워크스페이스의 `bag/`, `tracks/`, `results/`를 준비합니다. 소스는 AAC의 실제 분류 경로에서 직접 불러오므로 루트에 `tools/`, `scripts/`, `config/` 별칭을 만들지 않습니다. 녹화 데이터는 기존 AAC 이름인 `bag/` 하나를 사용합니다.

서브모듈은 원본의 읽기 권한이 필요할 수 있습니다. 원본 bag·대형 지도·가상환경·빌드 결과는 GitHub에 포함하지 않습니다. 소형 테스트 입력은 포함합니다. 위 검사는 소스 배치를 확인하며 의존성 설치·실차 연결 완료를 뜻하지 않습니다.

## 기존 본체에서 실행

```bash
cd ~/ros2_ws/src/aac_2026
bash operations/f1tenth/run.sh check
bash operations/f1tenth/run.sh python -m control.acados_mpcc --help
bash operations/f1tenth/run.sh tuner
bash operations/f1tenth/run.sh localization-test
```

기존 AAC PC 환경은 Humble, 가져온 F1TENTH 환경은 Jazzy입니다. 설치 경로를 섞지 않습니다. F1TENTH 빌드는 `ros2_ws/artifacts/f1tenth/<배포판>-<CPU>/`에 구분합니다. AGX 배포·실차 연결은 아직 수행하지 않았습니다.


기존 AAC Humble 환경의 실행은 [Humble 실행 안내](operations/f1tenth/docs/AAC_HUMBLE.md)를 따릅니다. 원본은 [Leedk3/aac_2026](https://github.com/Leedk3/aac_2026)이며 기존 노드·커밋 이력·서브모듈 출처를 유지합니다.
