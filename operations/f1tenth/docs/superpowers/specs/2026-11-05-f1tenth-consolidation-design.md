# F1TENTH 작업의 AAC 통합

사용자 요청: 본체와 Orin Nano에 나뉜 작업을 AAC 기존 구조에 모으고, 개발 도구와 과거 자료를 구분한다. 이후 AGX로 이전한다. 사용자가 직접 정리를 요청했으므로 별도 설계 승인 대화 없이 실행한다.

## 선택한 구조

`ros_ws/src/aac_2026`를 앞으로 편집할 소스의 기준으로 삼는다. 기존 AAC 노드는 유지하고 `sensors/`, `navigation/`, `planning/`, `control/`, `simulation/`, `operations/`, `docker/`에 기능별로 추가한다. PC 및 Nano 원본은 기존 실행과 복구를 위해 유지한다.

대안인 별도 `src/f1tenth` 저장소는 사용자가 AAC 내부 통합을 선택했으므로 적용하지 않는다. 전체 워크스페이스 복사는 데이터 중복과 아키텍처별 바이너리 혼입을 만들므로 적용하지 않는다.

## 가져올 것

- 본체: MPCC Python/C++·튜너·분석, 경로 생성, FAST-LIO·지도 검토·필터, bag 어댑터, Gazebo 재생.
- Nano: 실제 Livox 드라이버, 전방 필터, RealSense 드라이버·촬영 스크립트. 동일 이름의 본체 Livox 메시지 전용 패키지는 가져오지 않는다.
- 최근 별도 실험: 3D 위치 추정 코드·테스트·입력 및 의존하는 solver 어댑터. 오프라인 실험임을 명시한다.
- 기존 결과와 검증 설정은 데이터 연결과 출처 목록으로 보존한다.

## 보관과 환경

ROS1 Dynamic-LIO 원본, 이전 발표·CAD·실험 결과와 대형 bag은 기존 위치에 보존하고 `archive/f1tenth/`에서 찾을 수 있게 한다. 보관 폴더와 오프라인 도구는 colcon 탐색에서 제외한다. `build/install/log`, 가상환경과 컴파일된 solver는 소스로 복사하지 않는다.

본체 `ros_ws`에 기존 bags/results/tracks/acados 및 Python 환경의 로컬 연결을 둔다. 이는 로컬 재사용 설정이며 AGX 배포본으로 취급하지 않는다. AGX에서는 필요한 데이터와 소스, 의존성 버전을 옮기고 새로 빌드한다. Nano 실행·장치·네트워크·전원 설정은 변경하지 않는다.

PC·Nano·AGX의 역할과 확인된 버전을 `system/f1tenth/`에 기록한다. AAC의 기존 Humble 환경은 유지하며 가져온 F1TENTH는 Jazzy 기준이라고 구별한다. 실차 출력 연결·VESC 구현·AGX 배포는 이번 파일 정리의 범위에 포함하지 않는다.

## 검증

출처별 SHA256 목록으로 복사 누락과 원본 보존을 확인한다. Python의 `tools.*` 호환 import와 데이터 루트 탐색, 실제 Livox 소스, 깨진 링크, 중복 ROS 패키지, ROS1의 기본 빌드 유입을 확인한다. 기존 MPCC·계획·위치 추정 단위 검사를 실행하고 Jazzy 컨테이너에서 패키지 탐색 및 가능한 C++ 빌드를 확인한다. 하드웨어 주행이나 AGX 실행을 검증했다고 표현하지 않는다.
