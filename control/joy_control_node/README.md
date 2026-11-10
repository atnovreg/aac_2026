# joy_control_node

ROS2 node that converts PS4/PS3 joystick input into `AckermannDriveStamped` commands.  
Runs on a 100 Hz timer. Joystick input expires after 0.3 seconds by default.

---

## Topics

| Direction | Topic | Type | Description |
| --------- | ----- | ---- | ----------- |
| Sub | `/joy` | `sensor_msgs/Joy` | Raw joystick input |
| Sub | `/ackermann_auto` | `ackermann_msgs/AckermannDriveStamped` | Autonomous planner command |
| Pub | `/ackermann_drive` | `ackermann_msgs/AckermannDriveStamped` | Final drive command (100 Hz) |

> `/ackermann_auto` and `/ackermann_drive` are intentionally separate topics to prevent the node from receiving its own published messages as auto commands.

---

## Button Mapping (PS4)

| Button / Axis | Index | Function |
| ------------- | ----- | -------- |
| `buttons[1]` | Circle (○) | Enable RUN MODE on a fresh press |
| `buttons[0]` | Cross (×) | Software stop; wins over Circle |
| `buttons[5]` | R1 (hold) | Enable manual joystick control |
| `axes[4]` | Right stick vertical | Speed |
| `axes[0]` | Left stick horizontal | Steering angle |

---

## Operating Modes

```
joystick connected
│
├─ STOP (default)
│   └─ ○ button → RUN MODE
│
└─ RUN MODE
    ├─ R1 held                        → manual control from axes
    ├─ R1 released + /ackermann_auto  → pass-through autonomous command
    ├─ R1 released + no auto command  → speed=0, steer=0
    └─ × button                       → E-STOP → STOP
```

Zero commands are published before connection and after stale or malformed input.
Startup, input loss and Cross disarm the controller; release Circle and press it
again to rearm. Short messages or nonfinite used axes stop without crashing.
Axes are clamped to [-1, 1]. Cross is a software stop, not a hardware emergency stop.
With `manual_only:=true`, releasing R1 always outputs zero and auto input is ignored.
The legacy auto fallback is available only with `manual_only:=false`.

---

## Parameters

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| `maximum_speed` | `1.5` m/s | Maximum speed in manual mode |
| `maximum_steering_angle` | `0.524` rad (30°) | Maximum steering angle in manual mode |
| `reverse_angle` | `true` | Invert steering sign |
| `manual_only` | `false` | Ignore auto commands and stop when R1 is released |
| `input_timeout` | `0.3` s | Positive finite joystick freshness timeout |

---

## Launch

```bash
# Start joy_node + joy_control_node
ros2 launch joy_control_node joy_control.launch.py

# Specify joystick device index (default: 0 = /dev/input/js0)
ros2 launch joy_control_node joy_control.launch.py joy_id:=1
```

### Docker Notes

`joy_node` uses SDL2 and requires `SDL_VIDEODRIVER=offscreen` in headless environments.  
This is set in both `run_x86.sh` (`-e SDL_VIDEODRIVER=offscreen`) and the launch file.

The container user also needs access to `/dev/input/js*`. `run_x86.sh` passes the numeric input group GID via `--group-add $(getent group input | cut -d: -f3)` since the container image may not have an `input` group by name.

## Safety regression tests

Source the built ROS overlay, then run without any hardware driver:

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=177 python3 test/test_manual_safety.py
```

These integration tests start the real controller in an isolated local ROS domain.
They do not test motor braking, steering wiring or end-to-end hardware stopping.
