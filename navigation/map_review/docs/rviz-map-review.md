# 건지광장 지도 RViz 검토

실행: `bash src/aac_2026/navigation/map_review/scripts/rviz_map_review.sh` · 종료: `bash src/aac_2026/navigation/map_review/scripts/rviz_map_review.sh stop`

현재 대상은 2026-09-13 건지광장 `indoor_front180_20260913_083146`이다. bag 이름의 indoor는 환경 분류와 무관하다. 저장된 누적 지도를 보는 도구이며 bag 재생이나 SLAM 재실행을 하지 않는다.

## 왼쪽 Displays 체크박스

| 표시 | 내용 |
|---|---|
| 01 Static map | 필터링 후 정적 지도, 파랑, 기본 켜짐 |
| 02 Input map | 저장 자세로 누적한 입력, 회색 |
| 03 Static - hide floor | 정적 지도에서 추정 바닥 기준 높이 15cm 이하 제외 |
| 04 Input - hide floor | 누적 입력에 같은 높이 조건 적용 |
| 05 Recorded LIO trajectory | 당시 LIO 추정 궤적, 주황·빨강, 기본 켜짐 |

전체 지도 비교는 01과 02를 번갈아 켠다. 바닥 제외 비교는 01·02를 끄고 03·04를 번갈아 켠다. 05는 계속 켜 두면 위치를 알기 쉽다. 01을 켜둔 채 03을 추가하면 전체 지도의 바닥이 계속 보인다.

마우스 왼쪽 드래그는 회전, 가운데 드래그는 평행 이동, 휠은 확대·축소다. FlatColor 대신 Intensity 색상 변환을 선택하면 추정 평면 기준 높이를 색으로 볼 수 있다. 이 표시용 intensity는 반사 강도가 아니다.

## 데이터 범위와 해석

- 주행 궤적 XY 범위 +5m 구역, 공통 10cm 3D 복셀. 외곽 전체 원본 PCD와 해상도가 다르며 검토용이다.
- 누적 입력 826,838점, 정적 지도 233,773점. 바닥 제외 시 각각 418,149점과 147,322점. 궤적 3,374개 자세.
- 입력은 변환 bag의 모든 유효 스캔 중 저장 자세 시간 범위(3,373스캔)를 점별 시각에 보간해 누적했다. Dynamic-LIO 각도 투영 전 입력이다.
- 바닥 기준은 기존 검토에서 추정한 지배적 평면이다. 15cm 이하 연석 등도 숨겨질 수 있으므로 전체 지도와 함께 확인한다.
- review_world → review_map은 표시를 위한 항등 변환이다. 실제 차량 base_link 또는 센서 보정 TF가 아니다.
- 시각화를 위해 기존 자세 좌표에 review_map 이름을 부여했으며 지도 좌표 자체는 변경하지 않았다. 원본 지도와 bag는 변경하지 않는다.

## 구현·검증

`tools/map_review_cloud.py`: prepare / serve. 캐시와 출처: `results/건지광장/rviz_review/083146/`. RViz 설정: `src/aac_2026/navigation/map_review/config/map_review.rviz`.

ROS2 Jazzy 기존 Docker 이미지, ROS_DOMAIN_ID=77, 컨테이너 내부 LOCALHOST 발견을 사용한다. Transient Local + Reliable QoS로 처음에 지도를 한 번 전송하고 나중에 체크박스를 켠 구독자도 받을 수 있게 한다. workspace는 컨테이너에 읽기 전용 마운트한다.

검증: 경사진 평면에서 높이 필터와 바이너리 배치 단위 검사, 실제 ROS2 후발 구독자로 5개 토픽의 수신·점 개수·유한 좌표·바닥 제외 조건 확인, RViz 화면의 점군과 궤적 표시 확인.
