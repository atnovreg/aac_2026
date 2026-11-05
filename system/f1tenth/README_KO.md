# F1TENTH 데이터·장비 설정

본체와 Nano의 자료를 내용 해시로 대조한 결과입니다. 실제 장치 설정을 변경하거나 AGX에 배포한 것은 아닙니다.

| 파일 | 용도 |
|---|---|
| [공통 선택 자료](config/common/selected_assets.yaml) | 지도·경로·참고 제어 설정의 실제 위치와 해시 |
| [보정값 출처](config/common/calibration_sources.yaml) | 기존 factory-reference 설정과 미확인 실측값의 구분 |
| [Nano 조사 색인](config/nano/profile.yaml) | 저장된 주소·설정, 관측한 인터페이스·장치 상태 |
| [Nano MID360 설정](config/nano/MID360_config.json) | 실제 AAC 드라이버 JSON으로 연결되는 편집 경로 |
| [AGX 확인 항목](config/agx/profile.yaml) | 실제 AGX에서 확인할 주소·장치·장착 TF 등 |
| [AGX 자료 목록](deployment/agx_payload_manifest.json) | 가져갈 source·설정·선택 지도/경로와 선택 검증 bag |

`nano/profile.yaml`은 조사 결과를 정리한 색인입니다. 그 YAML의 값을 바꾸는 것만으로 ROS 드라이버에 적용되지는 않습니다. 센서 IP/포트는 `nano/MID360_config.json`이 연결하는 실제 드라이버 JSON에서 관리합니다. 기존 `msg_MID360_launch.py`가 읽는 JSON과 설치된 Nano 설정의 내용이 같은지 확인했습니다.

현재 저장값은 host `192.168.1.50`, LiDAR `192.168.1.137`이지만 조사 당시 센서 연결은 관측되지 않았습니다. 카메라 실측 보정, 차량 기준 센서 TF, VESC 구동/조향 보정과 AGX 설정은 미확정입니다. `navigation/fastlio/config/mid360_bag.yaml`은 `use_sim_time: true`인 녹화 데이터 매핑 설정으로, 완성된 실차 위치 추정 설정이 아닙니다.

추가 확보한 135개 설정·실험 기록은 본체 `ros2_ws/results/nano_import/20261105`에 있습니다. 이미 본체에 같은 내용이 있던 파일은 다시 전부 복사하지 않았으므로 이 폴더만으로 전체 과거 실험이 복제되는 것은 아닙니다.

전체 결과와 원본 위치 색인은 본체 `ros2_ws/artifacts/reconciliation/nano-20261105/REPORT_KO.md` 및 `evidence_index.json`에 있습니다. 삭제 파일별 이유와 keeper는 같은 폴더의 `cleanup_*_receipt.json`에 기록했습니다.

기존 `ros2_ws/bags`, `results`, `tracks`의 로컬 데이터 연결은 유지했습니다. 따라서 예전 `Workspace/f1tenth_ws`의 원본 데이터 폴더를 통째로 삭제하면 안 됩니다.
