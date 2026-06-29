# arduino_mega_control_node

Arduino Mega에 연결된 조향 DC motor, left in-wheel motor, right in-wheel motor를 제어하기 위한 기본 ROS2 C++ 노드입니다.

## Interface

| Type | Name | Message | Description |
| --- | --- | --- | --- |
| Sub | `/ackermann_drive` | `ackermann_msgs/AckermannDriveStamped` | 차량 속도와 조향각 명령 |
| Pub | `~/serial_connected` | `std_msgs/Bool` | serial 연결 상태 |
| Pub | `~/tx` | `std_msgs/String` | Arduino로 송신한 마지막 명령 문자열 |

## Serial Command

기본 프로토콜은 `/ackermann_drive`를 PWM 명령으로 변환하는 `pwm_line`입니다.

```text
<S:steering_pwm,L:left_pwm,R:right_pwm>
```

각 값은 signed PWM입니다. `*_pwm_min`, `*_pwm_max`, `*_direction`으로 하드웨어의 최소 구동 듀티, 최대 듀티, 방향을 config에서 맞춥니다.

이 모드를 쓰려면 Arduino Mega에도 같은 프로토콜을 파싱하는 firmware가 업로드되어 있어야 합니다. 기본 sketch는 `arduino/mega_pwm_control/mega_pwm_control.ino`에 있습니다. 기존 `F/B/L/R/G/I/H/J/S` 테스트 sketch가 보드에 남아 있으면 ROS 노드가 실행되어도 그 기존 sketch가 serial byte를 해석합니다.

Arduino sketch의 `F/B/L/R/G/I/H/J/S` 키 매핑 테스트는 `key_mapping_test_params.yaml`을 launch 파라미터로 넘겨서 사용합니다.

| Command | Meaning |
| --- | --- |
| `F` | forward |
| `B` | reverse |
| `L` | steering left only |
| `R` | steering right only |
| `G` | forward + left |
| `I` | forward + right |
| `H` | reverse + left |
| `J` | reverse + right |
| `S` | stop all |

## Run

```bash
ros2 launch arduino_mega_control_node arduino_mega_control.launch.py
```

키 매핑 테스트:

```bash
ros2 launch arduino_mega_control_node key_mapping_test.launch.py
```

USB 장치 이름이 다르면 `config/arduino_mega_control_params.yaml`의 `serial_port`, `usb_interface`, `baud_rate`를 수정하세요.
