# Jetson Manual Drive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Prepare and verify gamepad-to-VESC manual operation on the AGX, leaving unverified steering disabled.

**Architecture:** Reuse the existing joystick controller and upstream ROS2 VESC driver/converter. Add a small launch/config package and an operator script with dry-run, telemetry and drive stages; do not introduce a custom serial protocol.

**Tech Stack:** ROS 2 Humble, C++14, Python unittest/rclpy, upstream f1tenth/vesc ros2, ament/colcon.

**Spec:** `2026-11-11-manual-drive-design.md`

## Global Constraints
- No motor firmware update or motor calibration/protection changes.
- Input timeout 0.3 s; manual-only launch; driver maximum +/-2000 ERPM.
- Servo output stays preview-only until physical wiring/output capability/calibration are verified.
- Preserve unrelated changes and back up deployed files.

## Review Focus
- Joystick disappears while deadman is held: zero output and disarm, no stale auto fallback.
- Short/NaN joystick payload: zero output, process survives.
- Arm and stop pressed together or arm held during startup/reconnect: no unintended arming.
- Process dies: standard VESC driver sends no periodic ALIVE, allowing the tested firmware timeout to operate.
- Command scaling is provisional: hard electrical RPM limit, no claim of calibrated ground speed or steering.

### Task 1: Joystick safety
Files: `control/joy_control_node/{include/joy_control_node/joy_control.h,src/joy_control.cpp,test/test_manual_safety.py,README.md}`.
- [ ] Add real ROS integration tests for stale input/rearm, manual-only deadman release, stop priority, short/nonfinite payload, axis clamp and startup arm hold.
- [ ] Run against baseline ARM build and observe expected failures.
- [ ] Implement guards and parameters `input_timeout`, `manual_only`; rerun the full package safety suite.

### Task 2: Minimal manual launch
Files: `system/f1tenth_manual/{package.xml,CMakeLists.txt,launch/manual.launch.py,config/manual.yaml}`, `operations/f1tenth/manual.sh`.
- [ ] Add dry-run/telemetry/drive stages, local ROS domain, explicit serial by-id, bounded driver limits and preview-only servo routing.
- [ ] Build upstream VESC and local packages on ARM; verify actual topic routing and disarmed/held/released/stale output in dry-run.
- [ ] Inspect real DualSense input and VESC telemetry without sending motor/servo commands.

### Task 3: Deployment and record
- [ ] Back up replaced Jetson files, deploy only task files and build the selected overlay.
- [ ] Review the complete change, address material findings, rerun necessary tests.
- [ ] Record exact upstream revision, tests, installed commands and remaining physical steering/ground-drive work. Do not claim hardware drive validation from dry-run.
