# 3D 위치 추정 → MPCC 오프라인 실험

가상 3D 점군을 고정 지도에 ICP로 정합하고, 바퀴 속도·IMU 자이로로 센서 사이 자세를 예측합니다. 추정 결과가 MPCC 입력으로 들어갑니다. 실제 MID-360 수신·VESC 출력이 연결된 실차 노드는 아닙니다.

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
