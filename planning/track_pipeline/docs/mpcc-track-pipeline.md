# 실측 지도 → MPCC 트랙

현재 단기 구현은 `src/aac_2026/planning/track_pipeline/`이며 기본 입력은 `lidar_final`의 FAST-LIO2 결과입니다. 연구의 중심은 MPCC 재현·실패 조건·최소 수정 비교이고, 이 도구는 그 입력 트랙을 만드는 단계입니다.

## 생성과 확인

```bash
# 이미 설치된 호스트 Python 의존성 사용
python3 -m planning.track_pipeline --config src/aac_2026/planning/track_pipeline/config/track_extraction.yaml
# 같은 결과 경로를 다시 생성할 때만 명시
python3 -m planning.track_pipeline --config src/aac_2026/planning/track_pipeline/config/track_extraction.yaml --overwrite

# 기존 FAST-LIO Docker 이미지 위에 분석 의존성을 추가한 환경
# 최초에는 ./scripts/slam.sh build 로 기반 이미지를 생성
docker compose build track
docker compose run --rm -T track python3 -m planning.track_pipeline --overwrite
```

설정 파일의 `source_run`, `output`을 바꾸면 다른 bag 결과에도 적용할 수 있습니다. 관측상 닫힌 루프가 없으면 오류로 종료하며, 짧은 11초 bag을 인위적으로 폐회로로 만들지 않습니다. 기본 출력은 `tracks/indoor_final/`입니다.

## 파일과 의미

| 파일 | 내용 |
|---|---|
| `map.pcd` | 원본 PCD를 가리키는 상대 symbolic link. 원본 점을 수정하지 않음 |
| `bag_metadata.txt` | 원본 bag 메타데이터, 지도 실행 경로·보정 가정 |
| `trajectory.csv` | `t,x,y,z,yaw`; 수동 주행 LIO 궤적, 중심선이 아님 |
| `occupancy.pgm`, `occupancy.yaml` | 5cm conservative occupancy. 검정=관측 장애물, 흰색=바닥 관측, 회색=미관측. 낮은 높이 점도 모두 유지 |
| `map_evidence.npz` | floor/structural/low counts, observed_floor, usable, obstacles, origin, floor plane |
| `usable_area_boundary_*.csv` | 바닥 관측·구조벽에서 추론한 주행 영역의 외곽/내곽 contour. 각자 독립적인 표본 수이며 실제 벽 전체를 확정한 선은 아님. ROI 경계에서는 열린 contour일 수 있음 |
| `centerline.csv` | `s,x,y,yaw,curvature`; 지도 영역의 skeleton에서 고른 폐회로를 smoothing/resampling |
| `left_boundary.csv`, `right_boundary.csv` | `s,x,y,half_width`; **선택한 보수적 MPCC 트랙 제한선**. 실제 벽 위치와 구분 |
| `track.csv` | 중심선과 좌우 half-width 통합 |
| `track_liniger.mat` | MATLAB `track2.center`, `track2.inner`, `track2.outer`, 각각 2×N |
| `track_liniger.json` | C++ `X,Y,X_i,Y_i,X_o,Y_o` schema. C++ controller 실행은 별도 검증 필요 |
| `track_overlay.png` | 지도 영역 + 수동 궤적 + 중심선 + MPCC 제한선 |
| `evidence_review.png` | conservative occupancy와 구조 트랙 내부 낮은 높이 관측 비교 |
| `validation.json`, `extraction.yaml` | 검증 지표·정확한 추출 설정 |

좌표는 LIO `camera_init`의 XY, 단위 m/rad입니다. yaw는 [-π,π], curvature는 부호 있는 1/m입니다. 방향은 반시계 방향으로 통일해 left=inner, right=outer로 내보냅니다. CSV에는 중복 마지막 점을 넣지 않으며, 마지막 점과 첫 점을 연결해 폐회로를 구성합니다. 간격은 전체 길이/N으로 약 0.05m입니다. 바닥 plane은 높이 선택에만 사용하며 XY 좌표를 회전/스케일링하지 않습니다.

## 추출 원리와 가정

1. 궤적 근처에서 센서보다 0.4~1.3m 낮은 점으로 바닥 `z=ax+by+c`를 robust fitting합니다. 기본 지도에서 약 2.5cm의 95% fitting residual이 나왔습니다.
2. 바닥 ±0.1m의 점은 floor evidence로 사용합니다. 이 자료는 5개 중 1개 점을 사용합니다.
3. 현재 지도에서는 낮은 높이에 주행 루프 주변 누적 반환점이 많이 남습니다. 이동 물체인지 센서/플랫폼 반사인지는 확정하지 않았습니다. 구조 트랙은 바닥 위 2.0~2.5m 벽 구간을 사용하되, 낮은 높이의 **모든 원본 점**을 별도 obstacle evidence에 누적합니다. 낮은 관측을 무시해 conservative occupancy를 흰색으로 만들지 않습니다.
4. floor evidence를 0.15m dilation, 0.2m closing하고 2m² 이하의 작은 구멍을 추론 보완합니다. 이 usable mask는 관측된 free-space와 동일하지 않습니다. trajectory는 연결된 영역과 주행 루프 선택에만 사용합니다.
5. skeleton의 닫힌 cycle을 추출하고 주기 smoothing(0.8m), 등거리 resampling을 합니다.
6. 문/분기/코너에서 벽까지의 normal ray를 그대로 연결하면 제한선이 뒤로 접힐 수 있습니다. 그래서 map-area contour를 따로 보존하고, MPCC 제한선은 clearance에서 0.1m를 뺀 값, half-width 상한 0.75m, 곡률 반경의 70% 이하 조건을 적용한 내부 corridor로 만듭니다. 제한선은 차량 폭만큼 추가 축소된 선이 아닙니다.

현재 트랙은 **실내 구조 기반 시뮬레이션용 입력**입니다. 아래층 장애물 검토, 문의 개폐/통로 선택 확인, 실제 차량 footprint 검사가 끝나지 않았으므로 `ready_for_vehicle: false`입니다. 높은 벽 투영만으로 실차 free-space를 확정하지 않습니다.

## 검증

```bash
python3 -m unittest discover -s tests/track_pipeline -v
docker compose run --rm -T track python3 -m unittest discover -s tests/track_pipeline -v
```

합성 원형 복도에서 수동 궤적이 한쪽으로 치우쳐도 중심선이 지도 중앙에 생기는지 확인합니다. 폐곡선의 간격/곡률, 미관측 경계 처리, 교차 검출, CSV 표본 사이의 장애물 검출도 테스트합니다.

실측 결과는 중심선·좌우 선의 자체 교차, 좌우 상호 교차, 표본 사이를 포함한 전체 횡단면의 usable-mask 포함 여부를 검사합니다. 실패 결과도 검토를 위해 파일을 남기지만 `geometry_valid: false`와 nonzero exit code로 표시합니다. `geometry_valid`는 이 추론된 2D 영역의 기하 검증이며, 실제 장애물 부재나 차량 동역학 검증이 아닙니다.

## 기존 Liniger baseline 연결

로컬에서 찾은 baseline은 `alexliniger/MPCC` MATLAB 코드이며 ORCA 1:43 차량입니다. 이 상태로 F1TENTH 모델을 재현했다고 부르지 않습니다.

```bash
python3 src/aac_2026/control/acados_mpcc/scripts/run_liniger_track.py --baseline /path/to/alexliniger_MPCC --progress-speed 2
```

runner는 원본의 `Matlab/`을 `tracks/indoor_final/simulation/<UTC>/Matlab/`로 복사합니다. 변경 내역은 `integration.patch`, 출처는 `provenance.json`에 남깁니다.

- 실측 트랙을 원래 m 단위로 로드하고 global XY/progress bounds를 새 크기에 맞춤.
- 설치된 MATLAB `quadprog`, ego 1대, 기존 model/cost/dynamics를 사용.
- 원래 boundary shrink가 uniform width를 가정하므로 해당 전처리만 각 표본 폭으로 수정.
- 선택적으로 가상 진행 속도를 제한(`--progress-speed`, 실제 속도 명령과 동일하지 않음).
- solver 실패를 무시하지 않음. 실제 차량 pose와 그 pose의 track projection으로 한 바퀴 진행량을 판정.

초기 기본 조건에서는 65번째 step에서 `quadprog`가 iteration limit에 도달했습니다. 100→500 반복으로도 같은 지점에서 발생했으며 실패 결과를 보관했습니다. 단순히 파일을 읽었다는 이유로 lap 성공을 선언하지 않습니다. 각 run의 `summary.json`과 실제 궤적 검증 결과를 확인해야 합니다.

## 이번 실행 결과

- 폐회로 길이 53.062m, 1,062개 표본, 약 4.996cm 간격.
- 선택한 MPCC corridor 폭 0.833~1.500m. 물리적 복도 전체 폭과 다릅니다.
- 중심선/좌우 제한선 자체 교차 0, 좌우 상호 교차 0.
- 보간한 횡단면 481,466개 검사점 중 usable-mask 밖 0.
- 낮은 높이 관측이 있는 cell을 occupancy free로 내보낸 경우 0.
- 기본 진행 속도 설정의 MATLAB 실험: 65번째 step에서 quadprog raw status=0. 최대 500회 반복으로도 해결되지 않음.
- 가상 진행 속도 2m/s 제한 실험: 600번째 step에서 실제 pose 기반 진행량 23.87m, 614번째 step에서 raw status=0. 한 바퀴 미완료.

최신 실패 기록: [failure summary](../../../../../tracks/indoor_final/simulation/20260910T122729911268Z/failure_summary.json), [MATLAB log](../../../../../tracks/indoor_final/simulation/20260910T122729911268Z/matlab.log), [정확한 integration 변경](../../../../../tracks/indoor_final/simulation/20260910T122729911268Z/integration.patch). 이 결과를 adaptive MPCC의 한계나 실차의 실패로 해석하지 않습니다. 현재는 기존 baseline의 실측 트랙 integration 단계이며 QP 수렴 원인은 미확정입니다.

후속 [acados 이식](../../../control/acados_mpcc/docs/acados-mpcc.md)에서는 같은 ORCA 모델과 실측 트랙으로 SQP와 RTI 한 바퀴를 완료했습니다.
위 기록은 기존 MATLAB/quadprog 실행 결과로 유지하며, acados 실행의 계산 방법 차이는 별도 문서에 명시했습니다.
