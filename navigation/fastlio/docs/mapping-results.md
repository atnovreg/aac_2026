# FAST-LIO2 매핑 결과 — 2026-09-10

Docker/Jazzy 환경에서 원본 bag 5개를 각각 처리했습니다. 모두 프로세스 정상 종료, PCD 크기/포인트 수, 전체 XYZ 유한성, 궤적 timestamp 단조 증가와 quaternion 정규화를 검사했습니다.

**보정 가정:** MID-360 + 내장 IMU, upstream 기본 외부 보정값. 센서 모델/실측 보정은 사용자 확인 전입니다. 지도 정확도를 실측한 결과는 아닙니다.

| Bag | 기록 길이 (초) | 출력 자세/입력 스캔 | 지도 포인트 | PCD (MB) | 파일 |
|---|---:|---:|---:|---:|---|
| lidar_20260909_154004 | 132.5 | 1,322/1,326 | 7,424,224 | 237.6 | [PCD](../../../../../results/lidar_20260909_154004/20260910T112327624678Z/map.pcd) · [궤적](../../../../../results/lidar_20260909_154004/20260910T112327624678Z/trajectory.tum) · [미리보기](../../../../../results/lidar_20260909_154004/20260910T112327624678Z/preview.png) |
| lidar_2forward_1reverse_20260909_160207 | 11.4 | 111/114 | 644,089 | 20.6 | [PCD](../../../../../results/lidar_2forward_1reverse_20260909_160207/20260910T112233630243Z/map.pcd) · [궤적](../../../../../results/lidar_2forward_1reverse_20260909_160207/20260910T112233630243Z/trajectory.tum) · [미리보기](../../../../../results/lidar_2forward_1reverse_20260909_160207/20260910T112233630243Z/preview.png) |
| lidar_final_20260909_161208 | 431.4 | 4,312/4,315 | 24,706,945 | 790.6 | [PCD](../../../../../results/lidar_final_20260909_161208/20260910T112720950598Z/map.pcd) · [궤적](../../../../../results/lidar_final_20260909_161208/20260910T112720950598Z/trajectory.tum) · [미리보기](../../../../../results/lidar_final_20260909_161208/20260910T112720950598Z/preview.png) |
| lidar_retry_20260909_160305 | 384.6 | 3,843/3,849 | 21,852,301 | 699.3 | [PCD](../../../../../results/lidar_retry_20260909_160305/20260910T112924974871Z/map.pcd) · [궤적](../../../../../results/lidar_retry_20260909_160305/20260910T112924974871Z/trajectory.tum) · [미리보기](../../../../../results/lidar_retry_20260909_160305/20260910T112924974871Z/preview.png) |
| lidar_retry_20260909_162853 | 336.3 | 3,360/3,364 | 19,041,654 | 609.3 | [PCD](../../../../../results/lidar_retry_20260909_162853/20260910T113107477210Z/map.pcd) · [궤적](../../../../../results/lidar_retry_20260909_162853/20260910T113107477210Z/trajectory.tum) · [미리보기](../../../../../results/lidar_retry_20260909_162853/20260910T113107477210Z/preview.png) |

초기화와 마지막 IMU coverage 때문에 일부 스캔은 자세 출력이 없습니다. 각 출력 폴더의 `summary.json`, `config.yaml`, 실행 로그를 함께 보관했습니다.

짧은 bag은 0.5배속, 154004 bag은 2배속, 나머지는 4배속으로 실행했습니다. 테스트한 재실행 명령은 [README](../../../docs/history/workspace-before-organization.md)에 있습니다. 루프 폐합이나 bag 간 정합은 포함하지 않으며 각 지도 좌표계는 독립적입니다.

검증: C++ timestamp test, 실제 19,968-point 메시지의 어댑터 변환/잘못된 필드 거부, `/clock` 정지 상태의 처리 회귀 테스트 통과. FAST-LIO2 빌드에는 upstream CMake/Boost 경고가 있지만 컴파일은 성공했습니다.
