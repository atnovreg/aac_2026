# Jetson 수동 조작 준비

새 젯슨에 설치하거나 GitHub에서 받으려면 [빠른 시작](JETSON_QUICKSTART.md)을 따른다.

대상: AGX Orin / Ubuntu 22.04 / ROS Humble, USB DualSense,
FLIPSKY Dual FSESC6.7 Plus의 구동 채널 117 / 펌웨어 5.2.

## 젯슨 터미널에서 실행

```bash
~/f1tenth_ws/src/aac_2026/operations/f1tenth/manual.sh dry-run
```

기본값은 미리보기다. VESC 드라이버를 실행하지 않고
`/manual/erpm_preview`, `/manual/servo_preview`에만 변환 결과를 낸다.
수동 조작 패키지는 자동 주행·LiDAR·SLAM을 실행하지 않는다.
ROS_DOMAIN_ID=42, ROS_LOCALHOST_ONLY=1을 사용한다.

| 명령 | 기능 |
|---|---|
| `manual.sh setup` | Humble 의존성과 고정 VESC 소스를 준비하고 선택 빌드 |
| `manual.sh build` | 필요한 6개 ROS 패키지만 ARM 네이티브 빌드 |
| `manual.sh dry-run` | 게임패드와 명령 변환 미리보기. 하드웨어 드라이버 없음 |
| `manual.sh check` | 읽기 전용 VESC 검사 후 종료 |
| `manual.sh telemetry` | VESC 상태만 계속 수신. 제어 토픽은 연결하지 않음 |
| `manual.sh drive --wheels-up` | 읽기 검사 통과 후 바퀴를 띄운 구동 시험용 실행 |

`drive`는 차를 안정적으로 받쳐 구동 바퀴를 띄우고 직접 관찰할 때만 쓴다.
**조향은 모든 모드에서 미리보기만 가능하다. 지상 수동 주행 준비 완료가 아니다.**
여러 모드를 동시에 실행하지 않는다. 종료는 Ctrl+C.

## 조작

1. 모든 버튼과 스틱을 놓는다.
2. ○를 새로 눌러 주행 허용 상태로 바꾼다.
3. R1을 누르는 동안 오른쪽 스틱 위/아래로 전후진 명령을 낸다.
4. R1을 놓으면 속도 명령 0. ×는 주행 허용도 해제한다.
5. 입력이 0.3초 이상 끊기거나 잘못된 입력이 오면 0을 내고 해제한다.
   재연결 후 ○를 놓았다가 다시 눌러야 한다.

×는 소프트웨어 정지다. 속도 0 명령 또는 MCU의 기존 타임아웃이
실제 노면에서 즉시 정지하는 제동 거리까지 보장하지 않는다.
기존 MCU 타임아웃은 1000 ms, 타임아웃 제동 전류 0 A로 설정돼 있다.

현재 젯슨의 SDL 장치명은
`Sony Interactive Entertainment DualSense Wireless Controller`.
읽어 확인한 매핑은 ×=b1, ○=b2, R1=b5, 왼쪽 X=a0, 오른쪽 Y=a5다.
a4는 오른쪽 트리거이고 놓은 상태가 +1이다. 기존 PS4 기본 매핑을 쓰면 안 된다.
다른 패드/SDL 버전으로 바꾸면 실제 버튼·축을 다시 확인한다.

변환 계수 4614 ERPM/명목 m/s와 최대 명목 속도 0.3을 써서
현재 정상 입력의 상한은 약 ±1384 ERPM이다. 드라이버는 ±2000 ERPM으로
추가 제한한다. 바퀴 크기·기어비를 보정하지 않았으므로 실제 m/s로 해석하지 않는다.
서보 중심/끝점 계수도 실제 보정값이 아니다.

## 연결 검사와 남은 작업

USB 경로:
`/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00`.
이 이름은 두 채널을 구분하지 못하므로 `check`는 실제 상태의 controller_id=117,
오류 없음, 전압 10.2~12.65 V, 모터 정지, FET 70°C 미만을 검사한다.
이 검사는 셀별 전압·배터리 건전성을 판정하지 않는다.

모터 FOC/보호 설정과 펌웨어는 이 런처가 변경하지 않는다.
조향 서보의 전원/BEC·신호 경로, 공통 접지, 출력 기능, 중심과 끝점이
검증되기 전에는 서보 명령을 드라이버에 연결하지 않는다.
그다음 실제 패드 버튼/방향, 저속 구동·정지·USB/패드 분리 시 정지를
관찰한 뒤에만 지상 시험으로 넘어간다.

## 빌드 및 재현

필수 apt 패키지: `ros-humble-joy`, `ros-humble-ackermann-msgs`,
`ros-humble-serial-driver`, `ros-humble-asio-cmake-module`, `libasio-dev`.
사용자에게 `dialout,input` 그룹 권한이 필요하다. 그룹 변경은 새 로그인에 반영한다.

VESC 소스는 워크스페이스의 `src/vesc`에 둔다.
상류: https://github.com/f1tenth/vesc (ros2), 고정 커밋:
`153998df8545fe1781b975df88e411b4e71d4bfe`.
상류가 누락한 FET/모터 온도 필드 두 개를 채우는 패치를 빌드할 때 적용한다.
다른 커밋이나 충돌하는 소스에서는 빌드를 중단한다. 이 수정은 PC의 ROS
텔레메트리 전달만 바꾸며 MCU 펌웨어나 보호 설정을 쓰지 않는다.
최초 설치는 이 리비전을 체크아웃하고 `manual.sh build`를 실행한다.
현재 젯슨에서는 sudo 없이 누락된 asio CMake 패키지를 apt로 내려받아
`.manual_ros_deps/opt/ros/humble`에 풀었다. 빌드 스크립트가 이 경로를 인식한다.

선택 패키지 오버레이는 `build/manual`, `install/manual`, `log/manual`.
원본 조이스틱 패키지 백업은
`~/f1tenth_ws/experiments/manual_20261111/backup/joy_control_node-original`.

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/manual/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=177
python3 ~/f1tenth_ws/src/aac_2026/control/joy_control_node/test/test_manual_safety.py
python3 ~/f1tenth_ws/src/aac_2026/system/f1tenth_manual/test/test_dry_run.py
python3 ~/f1tenth_ws/src/aac_2026/system/f1tenth_manual/test/test_controller_check.py
python3 ~/f1tenth_ws/src/aac_2026/system/f1tenth_manual/test/test_driver_temperature.py
```

테스트는 실제 ROS 노드와 변환기를 사용하지만 하드웨어 드라이버를 실행하지 않는다.
실제 주행 시험 결과를 대신하지 않는다.
