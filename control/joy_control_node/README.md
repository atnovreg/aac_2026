# joy_control_node

ROS2 node that converts PS4/PS3 joystick input into `AckermannDriveStamped` commands.  
Runs on a 40 Hz timer, making it easy to add alternative input sources (e.g. keyboard) in the future.

---

## Topics

| Direction | Topic | Type | Description |
| --------- | ----- | ---- | ----------- |
| Sub | `/joy` | `sensor_msgs/Joy` | Raw joystick input |
| Sub | `/ackermann_auto` | `ackermann_msgs/AckermannDriveStamped` | Autonomous planner command |
| Pub | `/ackermann_drive` | `ackermann_msgs/AckermannDriveStamped` | Final drive command (40 Hz) |

> `/ackermann_auto` and `/ackermann_drive` are intentionally separate topics to prevent the node from receiving its own published messages as auto commands.

---

## Button Mapping (PS4)

| Button / Axis | Index | Function |
| ------------- | ----- | -------- |
| `buttons[1]` | Circle (○) | Enable RUN MODE |
| `buttons[0]` | Cross (×) | E-STOP (immediate stop) |
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

No messages are published until a joystick is connected.

---

## Parameters

| Parameter | Default | Description |
| --------- | ------- | ----------- |
| `maximum_speed` | `1.5` m/s | Maximum speed in manual mode |
| `maximum_steering_angle` | `0.524` rad (30°) | Maximum steering angle in manual mode |
| `reverse_angle` | `true` | Invert steering sign |

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
