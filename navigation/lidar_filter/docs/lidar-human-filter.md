# 이동체 관측점 제거 — 2026-09-11

대상은 현재 트랙에 사용한 `lidar_final_20260909_161208` bag과 기존 지도다. 원본 bag, 기존 지도, 기존 MPCC 트랙은 보존했다.

## 원인 확인

원본 스캔에서 사람 형태의 작은 군집이 센서 뒤 약 1~3m 사이에 나타난다. 센서가 거의 정지한 0~20초에도 군집이 약 1.2m 이동했다. 촬영자 본인인지 다른 사람인지 의미적으로 식별한 결과는 아니지만, 정적 벽이 아닌 이동 관측이라는 직접적인 근거다. 고정된 근거리 CropBox로는 코너 벽까지 제거할 수 있어 시간 비교 방식을 적용했다.

포인트 자체의 FLOAT64 epoch-ns 시각에 맞춰 TUM 위치를 선형 보간하고 자세를 SLERP로 보간했다. `camera_init <- IMU` 자세에 LiDAR-to-IMU offset을 적용한다. 여러 스캔의 상부 벽을 원본 지도에 대조한 오차 p95는 약 1.7~2.0cm였다. bag 수신 시각은 센서 시각보다 약0.1초 늦으므로 좌표 변환에 사용하지 않는다.

## 적용한 필터

- 원본 지도에서 10cm voxel 후보를 만들고, 약0.5초 간격의 777개 스캔 / 444만 개 실제 광선을 비교했다.
- 점이 안 보인다는 이유만으로 삭제하지 않는다. 실제 측정 광선이 해당 공간을 통과한 경우에만 빈 공간 증거로 사용한다.
- 광선 끝점 전방25cm 여유, 끝점 주변15cm 보호, 같은 프레임 중복 투표 제거를 적용했다.
- 최종 기준: 빈 공간 관측 최소10프레임, 빈 공간/(빈 공간+끝점 지지) 비율0.65 이상.
- 바닥 위0.15~1.9m만 제거 대상으로 삼는다. 상부 구조벽의 XY 근처20cm voxel은 추가로 보호한다.
- 센서 자세 범위 밖의 점, 판정 근거가 부족한 점은 유지한다.

검토 과정의 v1/v2도 보존하며, **최종 선택 결과는 `results/body_filter/20260911_visibility_v3/`**이다.

## 생성물

- `map_cleaned.pcd`: 기존 누적 지도에서 판정된 voxel의 점을 제외한 지도. 남긴 점의 XYZ/intensity 등 모든 필드는 그대로이며 좌표계와 기존 자세도 그대로다.
- `map_removed.pcd`: 분리한 관측점. 원본 지도343,843점이 여기에 포함된다.
- `lidar_final_20260909_161208_filtered/`: 필터링된 별도 rosbag. 4,315개 스캔에서1,001,091개의 원본 라이다 점을 제외했다. 지도는 이미 FAST-LIO의 다운샘플링을 거친 결과라 bag 제거점 수와 다르다.
- `evidence.npz`: voxel별 끝점/빈 공간 관측 횟수, 보호/제거 마스크.
- `manifest.json`, `summary.json`, `bag_summary.json`, `frame_audit.csv`: 입력·조건·개수·시간 보존 감사.
- `reference_validation.json`: 바닥·벽·사람 후보 영역의 독립적인 전후 비교.
- `before_after.png`, `corner_before_after.png`: 동일 원본 좌표와 동일 샘플링의 제거 전후 그림.

## 검토 결과와 한계

검토용으로 정의한 바닥, 상부 벽, 같은 사람 높이의 벽 영역에서 원본 점과8cm 점유 voxel이 모두100% 유지됐다. 사람 후보7개 작은 영역에서는 약75~95%의 점이 제거됐다. 이 기준영역의 보존율이 모든 정적 물체 보존을 보장하지는 않는다.

자동 사람 인식 모델을 사용한 것이 아니다. 시간에 따른 빈 공간 증거로 이동 관측 후보를 분리했다. 오래 가려졌거나 벽 가까이에 있는 잔상, 지정 높이 밖의 점은 일부 남을 수 있다. 이 결과만으로 실차 주행 가능 영역을 확정하지 않는다.

## 재현

```bash
docker compose run --rm -T track python3 -m tools.lidar_filter build \
  --output /ws/results/body_filter/NEW_RUN \
  --min-free-frames 10 --min-free-ratio 0.65 --min-static-frames 0

docker compose run --rm -T track python3 -m tools.lidar_filter export \
  --output /ws/results/body_filter/NEW_RUN

docker compose run --rm -T track python3 -m tools.lidar_filter.verify_bag \
  /ws/results/body_filter/NEW_RUN

./scripts/slam.sh run \
  /ws/results/body_filter/NEW_RUN/lidar_final_20260909_161208_filtered --rate 4

./scripts/gazebo_replay.sh --map-file \
  /ws/results/body_filter/20260911_visibility_v3/map_cleaned.pcd
```

별도 map-file은 트랙과 같은 camera_init 좌표계의 PCD여야 한다. 기존 누적 지도의 부분집합인 map_cleaned.pcd를 재생 배경으로 사용한다. 새롭게 최적화한 재매핑 지도는 자세가 달라질 수 있으므로 기존 트랙에 자동 교체하지 않는다.

이후 `map_cleaned.pcd`에서 중심선·경계를 다시 추출하고 새 acados RTI 주행 결과도 계산했다. 새 트랙은 `tracks/indoor_final_cleaned`, 새 실행은 `results/acados_mpcc/20260910T164333282308Z`다. [재계산 결과와 검증](../../../control/acados_mpcc/docs/cleaned-map-mpcc.md)

## 재매핑 확인

필터링한 bag으로 FAST-LIO를 다시 실행해4,315개 입력에서4,312개 자세와24,375,110점 지도를 생성했다. `results/lidar_final_20260909_161208_filtered/20260910T162006066345Z/`에 결과와 로그를 보관했다. 재생성 지도는 원본 기준 바닥/상부벽/낮은벽 voxel의12cm 이내 관측 보존율이 각각99.87%/99.92%/99.94%였다. 이는 정밀한 실측 정확도 검증이 아니다.

실제 MCAP을 다시 열어 남긴 라이다 기록이 원본의 순서 있는 바이트 단위 부분집합인지 확인하고,86,286개 IMU 메시지의 payload와 기록시간이 동일한지 검사했다. 원본과저장된IMU 해시가 일치한다.

rosbag2_py의 QoS 변환은 원본 UNKNOWN history의 depth0을depth10으로 기록한다. 이 정규화는 bag_validation.json에 별도로 기록하며, 나머지 QoS/메시지 metadata와 기록시간을 검사한다. FAST-LIO가 실제 받아들이는 마지막 점의 시각 변화는 최대1.275ms이며5ms 검사 한도 이내였다.
