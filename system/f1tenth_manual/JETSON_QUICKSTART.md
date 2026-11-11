# Jetson AGX Orin에서 PS5 수동 제어 실행

대상은 Ubuntu 22.04 + ROS 2 Humble이 설치된 AGX Orin이다.
현재 구동 모터용 설정과 PS5 명령 처리는 준비돼 있고, 조향은 하드웨어 담당자의
전원·출력 경로·중심·끝점 확인 전까지 미리보기 상태다.

## 처음 한 번: 소스 받기

비공개 저장소이므로 이 저장소에 접근할 수 있는 본인 GitHub 계정으로 인증한다.
GitHub CLI가 없다면 `sudo apt-get install -y gh`로 설치한다.

```bash
gh auth login --git-protocol https
mkdir -p ~/f1tenth_manual_release_ws/src
gh repo clone atnovreg/aac_2026 ~/f1tenth_manual_release_ws/src/aac_2026 -- \
  --filter=blob:none --no-checkout --single-branch --branch codex/jetson-manual-20261111
cd ~/f1tenth_manual_release_ws/src/aac_2026
git sparse-checkout set control/joy_control_node system/f1tenth_manual operations/f1tenth
git checkout codex/jetson-manual-20261111
bash operations/f1tenth/manual.sh setup
```

`setup`은 수동 제어에 필요한 apt 패키지를 설치하고, `src/vesc`가 없을 때만
지정된 상류 소스를 받아 고정 리비전으로 빌드한다. 기존 VESC 체크아웃의
리비전이 다르면 변경하지 않고 중단한다. 필요한 그룹을 새로 추가했다면
로그아웃·로그인 후 장치 시험을 진행한다.

선택 빌드 결과는 `~/f1tenth_manual_release_ws/install/manual`에 보존된다.
PC의 build/install 폴더를 복사하지 않는다.
이미 의존성과 권한이 준비된 장비에서는 `F1TENTH_SKIP_APT=1 bash operations/f1tenth/manual.sh setup`으로
apt 설치만 건너뛸 수 있다.

## 평소 실행

위 저장소 디렉터리에서 아래 명령을 사용한다.

| 명령 | 동작 |
|---|---|
| `bash operations/f1tenth/manual.sh dry-run` | PS5 입력과 명령 미리보기. 모터 드라이버 없음 |
| `bash operations/f1tenth/manual.sh check` | VESC 상태만 읽고 채널 117·오류·전압·온도 검사 후 종료 |
| `bash operations/f1tenth/manual.sh drive --wheels-up` | 상태 검사 후 바퀴를 띄운 저속 모터 시험 |
| `bash operations/f1tenth/manual.sh build` | 코드 변경 후 다시 빌드 |

종료는 Ctrl+C. 한 번에 하나의 실행 모드만 사용한다.
구동 시험은 차량을 받쳐 구동 바퀴를 띄우고 직접 지켜보는 상태에서 진행한다.
이 실행 명령은 조향 출력을 아직 연결하지 않는다.

PS5 조작은 ○=허용, R1 유지+오른쪽 스틱 위/아래=전후진, ×=정지 및 허용 해제다.
왼쪽 스틱은 조향 미리보기만 출력한다. 입력이 0.3초 이상 끊기면 해제되고,
입력이 돌아온 후 ○를 새로 눌러야 다시 허용된다.

기본 입력 설정은 이 AGX Orin에서 확인한 DualSense 매핑이다.
다른 SDL 환경에서 버튼 번호가 다르면 `F1TENTH_GAMEPAD_PROFILE`로 입력 프로필을
지정한다. PC Docker 실행은 `manual-pc.sh`가 PC 프로필을 자동 선택하므로,
그 환경 변수를 젯슨에 그대로 복사하지 않는다.

## 업데이트

```bash
cd ~/f1tenth_manual_release_ws/src/aac_2026
git pull --ff-only
bash operations/f1tenth/manual.sh build
```

코드·설정을 직접 수정했다면 먼저 변경 내용을 보존한다. 이 절차는 강제 초기화하지 않는다.

## 현재 범위

- VESC 펌웨어는 5.2 유지. 설정·FOC 측정값·보호값을 런처가 쓰지 않는다.
- ROS 모터 명령 한도 ±2000 ERPM. 명목 m/s 계수는 실제 차량 속도 보정값이 아니다.
- 조향 전원/신호/PWM 지원과 보정, 실제 제동 거리 및 지상 주행은 별도 확인 대상이다.
- PS5 USB 자동 재연결은 PC Docker에서 실제 검증했다. 젯슨에서의 실제 패드 및
  차량 전체 경로도 연결한 상태로 확인해야 한다.
- 현재 설치·실행은 수동 명령 방식이며 부팅 시 자동 주행 서비스는 만들지 않는다.

자세한 코드·시험 기록은 [README](README.md), [PC 검증](../../operations/f1tenth/docs/2026-11-11-pc-ps5-check.md)을 참고한다.
