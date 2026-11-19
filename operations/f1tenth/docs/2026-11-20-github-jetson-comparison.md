# GitHub 업로드와 Jetson 코드 비교 — 2026-11-20

## 업로드 기준

- 저장소: <https://github.com/atnovreg/aac_2026> (`PRIVATE`, `main`).
- 코드 커밋: `a08cdfc4ed62e3b41a489677727f728043597928`.
- 반영: 50개 파일, MPCC ROS 입력·계산/속도 요청, 라이다 기반 자동 위치 초기화, Jetson tmuxp 실행·기록, 차량 명령 선택기, 관련 설정·테스트·문서.
- 제외: 개인 `.vscode/settings.json`, 서브모듈 내부의 로컬 데이터 경로와 `COLCON_IGNORE`. 서브모듈 참조 커밋은 변경하지 않았다.
- 검증: MPCC 29개, 위치추정 23개, 실행 구성 36개, 차량 선택기 4개, 트랙 기하 5개로 총 97개 테스트 통과. 변경된 Python 21개와 셸 8개 문법 검사, 커밋 공백 검사 통과.
- ROS launch/센서 통합 테스트와 실차 주행은 이번 업로드 검증에 포함하지 않았다.

## 비교 방법과 결과 위치

Jetson에는 해당 비공개 GitHub 저장소의 인증이 없어 직접 clone은 실패했다. 본체에서 `git ls-remote`로 GitHub `main`과 위 커밋의 일치를 확인한 뒤, 그 커밋의 Git blob SHA256 목록과 변경된 50개 파일의 `git archive`를 Jetson으로 전달했다. GitHub 인증 정보는 전달하지 않았다.

Jetson의 비교 폴더:

```text
~/f1tenth_ws/github-review/20261120-c2yqyB/
  aac-20261120-manifest.json  # 전체 커밋의 파일별 해시와 서브모듈 참조
  aac-20261120-changed.tar    # 이번 커밋에서 변경된 50개 파일
  uploaded-changes/          # 위 변경 파일의 비교용 사본
  comparison.json           # 아래 각 위치의 동일/차이/없음 목록
  diffs/                    # 변경된 50개 파일 중 기존 파일과 다른 항목의 diff
```

기존 소스·배포 파일을 덮어쓰거나 차량 노드를 실행하지 않았다. 비교는 파일 내용 기준이며 실행 파일 권한·빌드 산출물·지도·경로 데이터·서브모듈 내부 내용은 검증하지 않았다. `diffs/`는 이번 변경 파일에 대한 상세 diff이며, 전체 차이 목록은 `comparison.json`에 있다.

## Jetson의 사본별 차이

기준은 GitHub 커밋의 일반 파일 1,500개다. 서브모듈 7개는 내용 비교에서 제외했다. 실행용 사본은 일부 파일만 배치한 구조이므로 아래의 '없음' 개수 자체가 실행 오류를 뜻하지 않는다.

| Jetson 위치 (`~/f1tenth_ws/` 기준) | 동일 | 내용 차이 | 없음 |
|---|---:|---:|---:|
| `src/aac_2026` | 1,422 | 42 | 36 |
| `live_indoor/runtime` | 27 | 4 | 1,469 |
| `live_indoor/indoor-drive` | 42 | 1 | 1,457 |
| `live_indoor/prepare-auto` | 34 | 5 | 1,461 |

`live_indoor/src/aac_2026`는 `live_indoor/runtime`을 가리키는 심볼릭 링크다. 별도 사본으로 세지 않는다. `src/aac_2026`에는 `.git`이 없으므로 해당 배포 사본의 원래 커밋은 확인하지 못했다.

### 먼저 확인할 기능 차이

1. **`runtime`의 MPCC는 기존 계산·기록 버전이다.** GitHub의 `ros_shadow.py`에는 0.1초 뒤 예측 속도를 이용한 속도 목표, 제한된 조향 요청, 선택적으로 `/mpcc/drive_request`를 발행하는 코드가 추가돼 있다. 관련 차량 선택기 파일도 `runtime`에는 없다.
2. **`runtime`의 위치추정에는 전역 자동 초기화가 없다.** GitHub의 launch와 `ros_localizer.py`는 `initial_base_pose=auto`와 입자 기반 초기화를 연결하며, `particle_localization.py`가 추가돼 있다.
3. **`indoor-drive`는 비교된 파일 중 기존 `indoor_tmux.sh` 사본 하나만 다르다.** 나머지 42개는 GitHub와 내용이 같다. 차이는 기록 준비 확인, trim 명령과 종료 후 trim 호출이다.
4. **최상위 배치 보조 스크립트는 이미 일치한다.** `live_indoor/indoor_tmux.sh`, `live_indoor/trim_indoor_run.py`는 각각 GitHub의 `operations/f1tenth/scripts/` 파일과 내용이 같다. 하위 사본의 스크립트와 구분한다.
5. **`prepare-auto`에는 별도 변경이 있다.** `ros_shadow.py`, 기존 `indoor_tmux.sh`, tmuxp의 `README.md`, `jetson.env`, `jetson.sh`가 GitHub와 다르다.
6. **일반 소스 폴더를 일괄 교체하지 않는다.** `src/aac_2026`의 차이에는 PS5 제어, acados 실행 경로, 기존 위치추정·표시·실행 도구도 포함된다. 어느 쪽 변경을 유지할지 상세 비교가 필요하다.

## Jetson에서 이어서 확인하기

다음 명령은 저장된 비교 결과만 읽는다.

```bash
cd ~/f1tenth_ws/github-review/20261120-c2yqyB
python3 -m json.tool comparison.json | less
less diffs/runtime/control/acados_mpcc/ros_shadow.py.diff
less diffs/runtime/navigation/localization3d/ros_localizer.py.diff
less diffs/indoor-drive/operations/f1tenth/scripts/indoor_tmux.sh.diff
```

다음 작업은 실제 실행 진입점이 `runtime`, `indoor-drive`, `prepare-auto` 중 무엇을 참조하는지 확인하고, 그 사본의 설정과 위 차이를 대조하는 것이다. 이번 작업에서는 실행 진입점을 변경하거나 새 버전을 배포하지 않았다. 비교 이후 파일이 바뀌면 저장된 해시 결과도 다시 계산해야 한다.
