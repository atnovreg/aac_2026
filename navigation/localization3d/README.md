# 3D localization → MPCC / 세 원 비교

사용자가 선택한3D 구성의 첫 기준 실험이다. 가상3D LiDAR가 측정한 XYZ 점군을
고정3D 지도에 점대면 ICP로 정합하고, 바퀴 속도·3축 IMU 자이로로 사이 시간을
예측한다. 이 추정 상태를 MPCC에 전달해 독립 Slash ODE 차량을 움직인다.
센서 생성·평가 외에는 GT를 사용하지 않는다. 센서나 정합에서2D 투영을 하지
않으며, 제어기에 필요한 평면 상태만3D 추정 결과에서 꺼낸다.

## 화면

- 파랑 원: GT 정답 위치.
- 주황 원: 지도 보정 없는 바퀴·IMU 오도메트리.
- 초록 원: 3D 정합으로 보정하고 센서 사이를 예측한 최종 추정 위치.
- 원의 반지름은 구분을 위한 표시 크기이며 공분산/오차 반경이 아니다.
- 차량·원은 XYZ와roll/pitch/yaw를 모두 갖는다. 궤적과 원의 중심은 실제 기록 좌표다.

가제보 컨테이너 `geonji-localization3d-display-20261104`는 결과를1배속 재생한다.
기존 두 바퀴 재생 컨테이너는 정지했으며 데이터는 그대로 보존했다.
화면은3D 센서 폐루프 계산의 기록을 재생한다. Gazebo 자체의 센서·접촉물리
또는 실시간 MPCC가 화면 안에서 실행 중이라는 뜻은 아니다.

```bash
docker start geonji-localization3d-display-20261104
docker stop geonji-localization3d-display-20261104
```

## 검증된 PC 결과

선택 실행: `results/20261104T131057591761Z`.
저속 목표1.5m/s에서112.06초 가상 시간에 한 바퀴(약164.74m)를 완주했다.
추정기 출력이 실제 MPCC 입력으로 전달됐는지 로그로 대조했다.
3D 정합1121/1121회 수락, GT의 원래 하드 영역 이탈0, 전체 기록 선분 포함.
차체·기존 임시 여유를 반영한 격자 여유의 최소 하한은0.1613m다.

이상적인 동일 voxel geometry 조건에서 위치 오차 RMS0.00180m, 최대0.00760m,
오도메트리 최종 오차2.87m였다. **이 숫자는 실제 MID-360 정확도가 아니다.**
센서 월드와 정합 지도가 동일한 근사 voxel geometry이므로 매우 유리한 기준
실험이다. 실제 지도 오차·가림·사람·반사율·보정 오차·측정 중 움직임은 별도다.

PC(x86_64)의 제어기 계산 평균4.49ms, 최대20.52ms였다. 센서 생성이 포함된
전체 반복에는 큰 지연이 있고 첫 반복에는 solver 빌드도 포함된다. 설정20ms나
평균값만으로50Hz 실시간 처리를 입증하지 않는다. 잿슨 결과는 별도로 보존한다.

## 센서·지도·추정 가정

- 원본 Dynamic-LIO083146의 측정3D 정적 점군을0.15m occupied voxel로 표현.
  계획 경계·완충띠·가정한 바닥 영역·수동 장애물은 레이캐스팅 물체로 쓰지 않는다.
- 수평360도 범위를240방향으로, 수직(-7..52도)을16방향으로 측정한다. 범위0.5..30m.
  첫 occupied voxel 면 반환, 거리 잡음 표준편차0.01m,2% 누락을 적용한다.
- 모든 점을 같은 시각에 측정한 generic instantaneous3D scan이다.
  MID-360 고유 스캔 패턴·점별 시간·motion distortion·반사율을 재현한 모델이 아니다.
- Wheel2% scale bias/0.01m/s 잡음, 자이로0.001rad/s z축 bias/0.002rad/s 잡음은
  실측값이 아닌 시나리오다. 가속도는 중력/운동에서 생성·기록하지만 이 기준
  추정기는 가속도 이중적분이나 tightly coupled LIO를 구현하지 않는다.
- 알고리즘은3D ICP+wheel/gyro 예측이다. FAST-LIO/Dynamic-LIO 실행으로 부르지 않는다.
- 초기 자세는 알려진 출발 영역 근처의 coarse prior다. 테스트에서XYZ에
  +0.2/-0.15/+0.08m 오차와 작은 회전 오차를 준다. 임의 위치 전역 재위치추정은 아니다.
- 3D 정합 불량·퇴화·추정 상태 만료를 감지하면 GT로 되돌리지 않고 시험을 중단한다.
- 목표 진행속도1.5m/s와 예측 전진속도 한계1.8m/s를 구분해 센서 잡음 여유를 둔다.
  원래 하드 경계/허용치는 유지하고 solver만0.03m 안쪽으로 제한한다.
- 횡속도는 추정 자세 변화에서 구해0.4s lowpass, wheel 속도는0.08s lowpass.
  경사면 body속도와gyro z를 그대로 평면 제어기에 넣지 않고 mapXY/yaw 기준으로 변환한다.

## 재현

PC에서는 원래 환경과 원본 모듈을 재사용한다. 프로젝트 원본 소스는 수정하지 않았다.

```bash
cd /home/irdl4/Documents/Codex/2026-11-04/task-4/localization3d
PYTHONPATH=. /home/irdl4/Workspace/f1tenth_ws/scripts/acados_python.sh -m unittest discover -s tests -v
PYTHONPATH=. /home/irdl4/Workspace/f1tenth_ws/scripts/acados_python.sh -W ignore::DeprecationWarning closed_loop.py --steps 7000
# 새 실행의 결과를 검증할 때 RUN경로를 지정
PYTHONPATH=. /home/irdl4/Workspace/f1tenth_ws/scripts/acados_python.sh audit.py --run results/20261104T131057591761Z
```

매번 새 timestamp 폴더에 source snapshot/config/provenance,3개SE3 궤적,
MPCC 입력·명령,20ms/4ms GT 기록,raw wheel/IMU,스캔 정합 진단을 저장한다.
raw3D scan은1초마다 저장하며 나머지는 기록한seed와geometry로 재생성한다.
`results/audit.json`, `results/three_pose_comparison.png`가 독립 검증/그림이다.

원본 pointcloud hash:
`7b4c61ea4129085579a1236f3ee29b427aec70b1846dfe90797f5b5008534789`.
사용된 모듈과 vehicle/model 해시는 각 실행의`provenance.json`에 남긴다.
모터·서보·실차 ROS 명령은 발행하지 않는다.

## 잿슨 검증과 현재 선택 결과

잿슨 Orin Nano Super의 기존 ARM64 컨테이너에서도 전체5603단계/112.06초
가상 한 바퀴를 완주했다. 현재 가제보 화면과 `results/audit.json`은 이 실행을
사용한다: `results/jetson/20261104T131654216541Z`.
PC 대비 GT/추정 위치 차이는 최대 약3e-10m이며, 원래 경계 이탈0과 격자 여유를
다시 확인했다. 파일 전송 SHA256도 확인했다. 원격 원본 결과는 그대로 남아 있다.

잿슨에서는 제어기 평균25.19ms/최대104.77ms, 정합 평균14.63ms/최대98.44ms였다.
센서 생성·추정·제어를 순차 처리하는 전체 반복 평균은74.00ms이며 초기 빌드가
포함된 최대값은27.19초다. 따라서 현재 구성은 **20ms 실시간 제어가 아니다.**
이후에 센서/추정의 비동기 처리와 제어 경로 최적화를 별도로 검증해야 한다.

현재 화면 검증:3개 모델의24개 pose 표본과 링의 실제 메시 중심까지 기록과
대조했다. 경사면에서 원 중심이 옆으로 이동하던 local-z 표시 오프셋을 제거했다.
시험14개가 통과했고, 원래 수치 실험에는 이 표시 수정이 영향을 주지 않는다.

주황 오도메트리 궤적이 벽/계획 영역 밖으로 나가는 것은 누적 위치 오차다.
이 궤적은 가상 차량의 실제 이동 경로가 아니다. MPCC는 초록색 보정 추정 상태를
받으며, 파란 GT 궤적을 독립적으로 검사했다. 거친3D 점군의 모든 점을 연속 충돌
표면으로 간주해 접촉물리까지 검증했다는 의미는 아니다.

잿슨 재실행:

```bash
ssh irdlf1tenth1@100.101.41.121 \
  'docker exec -e LOCALIZATION_WS=/workspace/f1tenth_ws -e BASELINE_EXPERIMENT=/workspace/f1tenth_ws/results/geonji_soft_sim_20261104 -w /workspace/f1tenth_ws f1tenth ./scripts/acados_python.sh -W ignore::DeprecationWarning results/geonji_localization3d_20261104/closed_loop.py --steps 7000'
```
