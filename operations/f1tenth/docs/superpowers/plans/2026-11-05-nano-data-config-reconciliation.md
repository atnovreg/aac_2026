# Nano Data and Configuration Reconciliation Plan

> **For agentic workers:** Use superpowers:executing-plans for implementation when the user requests execution. This document is the requested plan; do not begin transfer or deployment merely because it exists.

**Goal:** Nano에만 있는 필요한 지도·bag·로그·장치 설정을 대조해 보존하고, 본체 AAC의 기준 자료와 AGX 배포에 필요한 최소 자료를 확정한다.

**Architecture:** 본체와 Nano의 원본은 유지한다. 메타데이터 목록 작성 → 실제 내용·출처 비교 → 별도 staging으로 선택 전송 → 검증 → 기존 bags/tracks/results 및 AAC 설정에서 사용할 기준 확정 순서로 진행한다.

**Tech Stack:** SSH read-only inspection, Python, SHA256, ROS2 bag metadata/info, YAML/JSON, filesystem staging.

**Spec:** `docs/superpowers/specs/2026-11-05-f1tenth-consolidation-design.md` 및 사용자가 추가 요청한 Nano의 데이터·설정 대조 범위.

## Global Constraints

- 본체 기준 작업공간: `/home/irdl4/ros2_ws`, 소스: `src/aac_2026`.
- 본체 bags/results/tracks의 실제 저장 위치는 기존 `Workspace/f1tenth_ws`를 가리키는 로컬 링크임을 고려한다.
- Nano 원본: `/home/irdlf1tenth1/f1tenth_ws`; 기존 Docker `f1tenth`와 파일·장치 설정은 변경하지 않는다.
- 수정시각·이름만으로 최신/정답을 결정하지 않는다. 내용 해시와 실행 출처를 대조한다.
- 기존 파일 덮어쓰기, 삭제 동기화, Nano 네트워크·전원·센서·VESC 변경은 하지 않는다.
- 현재 기록 중인 bag·로그는 완성 파일과 구분하고, 일관된 복사본을 얻기 전에는 전송 완료로 표시하지 않는다.
- 본체의 기존 원본·새 staging·Nano 원본을 서로 구별한다. 검증이 끝나기 전에 기준 경로를 바꾸지 않는다.
- AGX 환경과 새 장착 위치의 주소·TF·보정값은 미확정으로 남긴다. Nano의 값을 검증 없이 AGX 확정값으로 복제하지 않는다.
- `build/install/log`, 가상환경·SDK 설치 바이너리 전체는 배포 자료로 복사하지 않는다.
- 사용자 후속 승인에 따라 내용이 완전히 대조된 중복 압축본, 종료된 PID 표식, 원본 `.py`가 있는 bytecode 및 이 작업의 검증 완료 임시 파일은 정리한다. raw bag·source·현재 설치 환경은 유지하며 삭제 이유와 keeper를 기록한다.

## Initial Findings (2026-11-05, read-only)

- 본체와 Nano의 `results/건지광장/map_edit/current_selection.json`은 모두 `20261001_similarity_obstacles`를 선택하지만 `static_cloud`와 `satellite_registration` 경로가 서로 다르다. 동일 지도인지 내용 비교가 필요하다.
- Nano bags에서 metadata.yaml 10개를 확인했다. 기존 본체 bag과 중복 여부는 아직 해시 대조하지 않았다.
- Nano results에는 3D 위치 추정·soft simulation·two-lap 실험과 camera 자료가 있다. 일부 결과를 앞서 본체에 받아둔 기록이 있으므로 다시 전부 복사하지 않는다.
- Nano Livox config에는 MID360 설정과 `.bak`가 공존한다. 실행 launch가 참조하는 실제 파일을 확인해야 한다.
- 기존 C++ 선택 기록은 가상 차량 벤치마크이며 물리 차량 검증 기록으로 사용하지 않는다.

## Review Focus

- 이름이 같은 지도지만 점군·좌표계·처리 단계가 다른 경우.
- 선택 JSON/launch/config 경로와 실제 실행에서 사용한 값이 다른 경우.
- 기록 중인 bag의 metadata와 chunk가 복사 도중 달라지는 경우.
- 보정값이 실측값인지 가정값인지, 이전 장착 위치에 종속되는지 불명확한 경우.
- 상대 링크와 로컬 절대 경로 때문에 AGX 자료 목록만 옮겨서는 실행되지 않는 경우.

### Task 1: 목록과 실행 출처 확보 — 본체에서 Nano를 읽기 전용으로 조사

- [x] 본체와 Nano의 bags/tracks/results/config 및 장치 설정 후보를 목록화한다. 경로·크기·파일형식·원본/링크·실행/측정 출처를 기록한다.
- [x] 현재 기록 중인 bag/log가 있는지 실행 프로세스와 파일 상태로 확인한다. 오래된 `.pid` 파일만으로 활성 상태를 판정하지 않는다.
- [x] launch 인자, 실제 설정 파일, 센서 IP/host IP/포트/인터페이스, USB 식별자·udev, ROS domain/RMW, TF와 시간 설정의 출처를 확인한다.
- [x] 결과를 `artifacts/reconciliation/nano-20261105/inventory.json`과 `runtime_config_sources.json`으로 남긴다.

**Expected:** 실제 위치와 후보 목록이 생기며 원격 설정 변경이나 대용량 데이터 전송은 없다.

### Task 2: 중복·다른 버전·기준 자료 구분 — 본체에서 비교

- [x] 작은 설정·선택 정보부터 SHA256/내용을 비교하고, 필요한 지도와 종료된 bag/log로 비교 범위를 넓힌다.
- [x] 지도는 점군 해시·점 수·좌표계·생성 출처·변환/편집 기록과 선택 경로를 함께 확인한다.
- [x] bag은 metadata의 토픽·타입·기간·메시지 수와 모든 실제 chunk를 비교한다. metadata.yaml만으로 동일하다고 판단하지 않는다.
- [x] 같은 내용, Nano에만 있음, 본체에만 있음, 이름은 같고 내용은 다름, 확인 필요로 분류한다.
- [x] 서로 다른 파일은 실행 provenance와 의존 관계로 판단하고, 근거가 부족하면 두 버전을 보존한다.

**Expected:** `comparison.json`과 지도·경로·설정 선택 근거가 생긴다. 현재 선택 포인터는 아직 바꾸지 않는다.

### Task 3: 필요한 자료만 선택 전송 — Nano → 본체

- [x] 1순위: 기준 지도·경로와 그 의존 파일, 실제 사용 설정·보정 출처, 아직 본체에 없는 검증 로그.
- [x] 2순위: 재현에 필요한 대표 bag과 Nano에만 있는 원본 기록. 큰 원본은 목록과 용량부터 확정한 뒤 순차 전송한다.
- [x] 기존 자료와 충돌하지 않는 `artifacts/reconciliation/nano-20261105/staging/`에 원본 상대 경로와 출처를 유지해 받는다.
- [x] 전송 전후 해시·크기·완료 상태를 기록한다. 전송 중 원격 파일이 바뀌면 일관된 스냅샷으로 다시 확인하기 전까지 보류한다.

**Expected:** 중복을 제외한 선택 자료가 본체 staging에 있고 `transfer_manifest.json`으로 원본과 동일함을 확인할 수 있다.

### Task 4: 공통 설정과 장비별 설정 정리 — 본체 AAC

- [x] 확정한 지도·경로·모델·공통 좌표계 규격과 Nano 전용 IP/USB/인터페이스 설정을 구분한다.
- [x] AAC `system/f1tenth/config/`의 하위 공통/장비 프로필로 정리하되 기존 코드·launch가 읽는 경로와 연결한다. 단순히 설정 파일을 복사해 놓고 적용됐다고 판단하지 않는다.
- [x] 실측·제조사 값·가정값·미확인값을 구별하고 보정 파일의 출처·적용 장착 상태를 기록한다.
- [x] AGX 프로필에는 실제 확인한 항목만 넣고 주소·포트·장착 TF 등을 임의로 확정하지 않는다.
- [x] 검증된 자료만 기존 bags/tracks/results의 충돌 없는 위치에 반영하고 선택 manifest를 만든다. 나머지는 출처를 유지해 보관한다.

**Expected:** 본체에서 편집하는 기준 설정과 Nano/AGX별 미확정 항목이 분명해진다.

### Task 5: 재현 확인과 AGX 배포 목록 확정

- [x] 전송 해시·모든 지도/경로 참조·상대 경로·장비 프로필의 연결을 검사한다.
- [x] 본체에서 ROS bag metadata/info와 필요한 구간의 가상 입력 검사를 수행한다. 모터 명령을 실제 장치로 송신하지 않는다.
- [x] 비교한 자료의 실제 사용 결과와 남은 불일치·미확정값을 보고한다.
- [x] AGX에 가져갈 소스·설정·선택 지도/경로·대표 검증 입력의 최소 manifest를 만든다. 나머지 bag과 과거 로그는 본체 보관 대상으로 표시한다.
- [x] 실제 장치와의 센서/타임스탬프/보정/제어 연결 검증은 별도 Jetson 실행 단계로 남긴다.

**Expected:** `verification.json`, `agx_payload_manifest.json` 및 사람이 읽을 수 있는 대조 요약. 본체의 파일 확인과 실제 AGX 주행 검증을 구분한다.
