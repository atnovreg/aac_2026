# PC Docker 실행 환경

2026-11-11 PC 환경 준비. 기존 `d4d-koyul:humble-fortress` 이미지의
Ubuntu 22.04 / ROS Humble / x86_64 환경을 재사용하고 수동 제어 의존성을 추가했다.
이미지 정의는 이 디렉터리의 `Dockerfile.pc`다. 기반 이미지는 이 PC의 로컬 이미지다.

검증 결과: x86_64 패키지 6개 빌드 성공, PC/젯슨 두 입력 프로필을 포함해
자동 테스트 18개 통과.
기본 미리보기의 joy/controller/converter 노드도 실행했고, 입력 없는 상태에서
ERPM 출력 575개가 모두 0인 것을 확인했다. 실제 PS5로 전후진 명령, R1 해제,
× 정지, ○ 재허용, USB 분리 후 0 유지, 재연결 후 ○ 전 차단도 확인했다.
후속 보완으로 입력 노드·제어 노드를 재시작하지 않은 자동 USB 재연결도 검증했다.
차량이나 VESC를 연결해 모터를 구동한 시험은 아니다.

- 전용 이미지: `aac-f1tenth-manual:humble-pc`
- 소스: `/home/irdl4/.cache/codex-worktrees/aac-manual-20261111`
- PC 전용 빌드·설치: `/home/irdl4/f1tenth_manual_pc_ws/{build,install}/manual`
- 결과 로그: `/home/irdl4/f1tenth_manual_pc_ws/verification/`
- 외부 VESC 소스: 위 PC 워크스페이스의 `src/vesc`, 리비전은 기존 manual.sh와 동일.

PC 소스는 컨테이너의 `/ws/src/aac_2026`에 읽기 전용으로 마운트한다.
호스트 워크스페이스의 `src/aac_2026` 디렉터리가 비어 있는 것은 이 마운트 구성 때문이다.
빌드 결과는 호스트에 보존되며, 새 컨테이너에서도 같은 `/ws` 경로로 접근한다.

## 재빌드

```bash
docker run --rm --network none --user "$(id -u):$(id -g)" \
  --entrypoint bash --workdir /ws \
  -e F1TENTH_WS=/ws -e ROS_HOME=/tmp/manual-ros -e ROS_LOG_DIR=/tmp/manual-ros/log \
  -e F1TENTH_GAMEPAD_PROFILE=/ws/src/aac_2026/system/f1tenth_manual/config/manual.pc.yaml \
  -v /home/irdl4/f1tenth_manual_pc_ws:/ws \
  -v /home/irdl4/.cache/codex-worktrees/aac-manual-20261111:/ws/src/aac_2026:ro \
  aac-f1tenth-manual:humble-pc \
  -lc 'bash /ws/src/aac_2026/operations/f1tenth/manual.sh build'
```

## 하드웨어 없는 미리보기

```bash
docker run --rm -it --network none --user "$(id -u):$(id -g)" \
  --entrypoint bash --workdir /ws \
  -e F1TENTH_WS=/ws -e ROS_HOME=/tmp/manual-ros -e ROS_LOG_DIR=/tmp/manual-ros/log \
  -e F1TENTH_GAMEPAD_PROFILE=/ws/src/aac_2026/system/f1tenth_manual/config/manual.pc.yaml \
  -v /home/irdl4/f1tenth_manual_pc_ws:/ws \
  -v /home/irdl4/.cache/codex-worktrees/aac-manual-20261111:/ws/src/aac_2026:ro \
  aac-f1tenth-manual:humble-pc \
  -lc 'bash /ws/src/aac_2026/operations/f1tenth/manual.sh dry-run'
```

위 명령은 USB 장치를 컨테이너에 전달하지 않으므로 실제 PS5 입력은 들어오지 않는다.
이 명령 자체는 하드웨어 없는 미리보기다. 실제 PS5 입력을 사용하려면
아래의 입력 장치 전달과 PC 프로필이 필요하다.
VESC·차량과 연결되지 않으며 조향도 미리보기 토픽에만 출력한다. 종료는 Ctrl+C.

자동 테스트는 같은 컨테이너에서 Humble 및 `/ws/install/manual/setup.bash`를
source하고 `ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=177`로 실행한다.
기존 README의 4개 Python 테스트 파일을 사용한다.
온도 전달 시험은 물리 장치 대신 가상 직렬 포트를 사용한다.

## 실제 PC 패드 매핑

이 PC에서 읽은 SDL 매핑은 ×=b0, ○=b1, R1=b5, 왼쪽 X=a0, 오른쪽 Y=a4다.
젯슨의 b1/b2/a5와 달라 `config/manual.pc.yaml`을 입력 설정에만 덧씌운다.
공통 전류/회전수 제한과 젯슨 기본 설정은 그대로 유지한다.
직접 launch를 쓸 경우 `gamepad_profile:=<PC 설정 파일 경로>`를 지정한다.
파일이 없으면 실행을 중단한다.

## PS5 실제 입력과 USB 재연결

```bash
bash /home/irdl4/.cache/codex-worktrees/aac-manual-20261111/operations/f1tenth/manual-pc.sh
```

기존에는 event27/js0을 각각 마운트했기 때문에 USB 재연결 때 입력 노드를 다시
열어야 했다. 새 실행 스크립트는 `/dev/input` 디렉터리를 읽기 전용으로 전달하고
입력 장치 major 13의 읽기 권한과 호스트 input 그룹을 사용한다. 장치 번호가
바뀌어도 디렉터리 생성/삭제 이벤트가 보인다. VESC 직렬 장치는 전달하지 않는다.

`SDL_JOYSTICK_DISABLE_UDEV=1`로 SDL의 디렉터리 감시 경로를 사용한다.
[사용 중인 SDL 2.0.20 소스](https://github.com/libsdl-org/SDL/blob/release-2.0.20/src/joystick/linux/SDL_sysjoystick.c)는
이 경로에서 `/dev/input`의 생성/삭제/권한 변경을 감시한다.
PC 입력 프로필도 스크립트가 자동 선택한다. 종료는 Ctrl+C.

이 PC에는 같은 스크립트로 연결된 `~/f1tenth-manual-pc` 실행 링크도 준비했다.

입력이 0.3초 이상 끊기면 기존 제어 로직이 해제하며, 복구 후 ○를 새로 누르기
전까지 명령을 차단한다. 실제 시험에서 약 10.6초 입력 공백 후 같은 컨테이너와
동일한 입력/제어 프로세스가 자동 복구했다. ○ 전 R1+스틱 입력에도 출력 0,
새 ○ 이후 명령 재개를 확인했다. 결과는 PC verification의 `hotplug-result.json`에 있다.

PC 프로필 회귀 시험은 기존 `test_dry_run.py`를 실행할 때
`MANUAL_TEST_GAMEPAD_PROFILE=/ws/src/aac_2026/system/f1tenth_manual/config/manual.pc.yaml`
환경 변수를 추가한다. 변수 없는 시험은 기존 젯슨 프로필을 검증한다.
