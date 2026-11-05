# 정리한 지도 기반 acados MPCC 재계산 — 2026-09-11

`map_cleaned.pcd`에서 중심선과 좌우 제한선을 새로 추출하고, acados SQP_RTI로 새 주행 결과를 계산했다. 기존 결과는 보존했다.

## 입력과 결과

- 지도: `results/body_filter/20260911_visibility_v3/map_cleaned.pcd`. 기존 FAST-LIO 지도의 좌표를 유지한 24,363,102점 부분집합이다.
- 입력 묶음: `results/lidar_final_20260909_161208/20260911_cleaned_visibility_v3_track_input/`. 정리 지도와 원본 `trajectory.tum`을 상대 링크로 연결했다. 새 FAST-LIO 실행으로 가장한 폴더가 아니며 출처·해시는 `input_manifest.json`에 기록했다.
- 트랙: `tracks/indoor_final_cleaned/`. 기존과 동일한 추출 파라미터로 다시 계산했다. 원본 LIO 궤적은 영역·루프 선택 참고이며 새 중심선으로 복사하지 않았다.
- acados 실행: `results/acados_mpcc/20260910T164333282308Z/`.
- Gazebo: `results/gazebo_replay/20260910T164430122440Z/`. 새 트랙과 새 주행 기록의 해시를 확인하고 재생했다.

이번 입력은 정리한 원본 지도의 부분집합이다. 필터 bag을 FAST-LIO로 다시 처리한 별도 재매핑 지도와 그 자세를 섞지 않았다.

| 항목 | 기존 Docker RTI | 새 RTI |
|---|---:|---:|
| 중심선 길이 | 53.0623 m | 53.1408 m |
| 중심선 표본 수 | 1,062 | 1,063 |
| 선택한 트랙 최소 폭 | 0.8326 m | 0.9660 m |
| 한 바퀴 시간 | 13.88 s | 13.88 s |
| 제어 주기 수 | 694 | 694 |
| 중간 상태의 경계 이탈 | 0 m | 0 m |
| solver 오류 횟수 | 0 | 0 |

차량 모델은 ORCA 1:43이며 기존 RTI와 모델·controller·track 코드 및 제어 파라미터가 같다. 실행 설정의 차이는 입력 트랙 경로다. 이전 실행과 `simulation.py` wrapper 해시는 다르므로 전체 프로그램 소스가 완전히 같다고 주장하지 않는다. 이번 작업에서는 solver·모델 소스를 수정하지 않았다.

## 검증

- 새 트랙의 자체/상호 교차 0, 횡단면 489,528개 검사점 중 추론 영역 밖 0, `geometry_valid: true`.
- 별도 검증 스크립트가 저장된 진행량이나 `Track.project`를 쓰지 않고 전체 중심선 spline을 1 mm 간격으로 재구성해 3,470개 중간 차량 상태를 전역 투영했다.
- 독립 계산한 최종 진행량 53.1822938 m로 한 바퀴 완료를 확인했다. 저장된 진행량과의 최대 차이는 0.214 mm다.
- 경계 이탈 0 m. 차량 폭·안전계수에 따른 0.045 m 여유를 반영한 뒤 최소 잔여 횡방향 여유는 0.1174 m다.
- 상태·입력 제약 최대 수치 초과 0.0002646은 설정 허용오차 0.002 이내다. 입력 변화량 위반 0, solver 상태 694개 모두 0이다.
- 실제 XY 이동거리 합은 51.8535 m다. 코너 안쪽으로 주행하므로 중심선 길이·진행량과 구분한다.
- 기존 트랙·설정 파일 해시 보존 및 새 입력 해시 일치를 포함해 독립 검증 27개가 통과했다. 기존 트랙 테스트 5개도 다시 통과했다.

[독립 검증 결과](../../../../../results/acados_mpcc/20260910T164333282308Z/independent_validation.json) · [검증 스크립트](../../../../../results/acados_mpcc/20260910T164333282308Z/independent_validate.py)

Solver 평균/p95/최대는 0.932/1.110/1.497 ms였다. Python을 포함한 controller cycle은 평균 2.445 ms, 최대 22.089 ms로, 20 ms 주기의 엄격한 실시간 달성이나 Jetson 성능을 보장하는 결과가 아니다.

## 그림과 재현

[새 트랙](../../../../../tracks/indoor_final_cleaned/track_overlay.png) · [새 주행 결과](../../../../../results/acados_mpcc/20260910T164333282308Z/simulation.png) · [이전 경로와 비교](../../../../../results/acados_mpcc/20260910T164333282308Z/path_comparison.png) · [Gazebo 화면](../../../../../results/gazebo_replay/20260910T164430122440Z/gazebo_cleaned_new_mpcc.png)

바탕화면 `20260911` 폴더에도 10~13번 그림으로 복사했다. 이전 08·09번은 정리 지도에 **기존** 주행 기록을 재생한 화면이고, 13번은 **새로 계산한** 기록을 재생한 화면이다.

실행한 명령:

```bash
python3 -m tools.track_pipeline --config config/track_extraction_cleaned.yaml
docker compose run --rm -T acados python -m tools.acados_mpcc \
  --config config/acados_mpcc_rti.yaml --track tracks/indoor_final_cleaned
./scripts/gazebo_replay.sh --run /ws/results/acados_mpcc/20260910T164333282308Z
python3 results/acados_mpcc/20260910T164333282308Z/independent_validate.py
```

트랙 추출기는 기존 출력 덮어쓰기를 기본 거부하므로 재추출할 때는 설정의 `output`을 새 폴더로 바꾼다. acados는 매번 새 실행 폴더를 만든다.

기존 대비 경로 차이는 작다. 5 mm 간격 폴리라인의 양방향 최근접 거리 기준 새/기존 주행 경로의 평균 차이는 약 1.20 cm, 최대는 10.84 cm다. 점 제거로 표본 인덱스가 달라지고 바닥 평면도 다시 추정하므로 이 차이를 사람 흔적 제거 효과로만 해석하지 않는다. 구조 트랙은 바닥과 2~2.5 m 벽을 사용하며 낮은 높이 관측은 별도 occupancy에 남긴다.

이 결과는 ORCA 모델의 오프라인 시뮬레이션 및 Gazebo 기록 재생이다. 모든 낮은 장애물에 대한 충돌 회피, F1TENTH 차량 모델·장착 TF·실시간 위치 추정·ROS2 제어 연결은 아직 검증하지 않았다.
