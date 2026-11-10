# Jetson gamepad manual drive

Goal: prepare the AGX Orin (Ubuntu 22.04, ROS Humble) for low-speed gamepad operation using its existing DualSense and calibrated VESC 5.2. The operator authorized continuous preparation; hardware motion remains an announced, wheels-raised test.

Reuse `joy_control_node`, the ROS `joy` package, and upstream `f1tenth/vesc` ROS2 packages. Repair the joystick node's stale-input behavior before using it. The manual launch disables automatic-command fallback. ROS traffic stays on localhost in a separate domain.

Requirements:
- No motor firmware update or motor calibration/protection changes.
- Hold R1 to move, Circle to arm, Cross has priority to stop.
- Stop/disarm after 0.3 s without valid joystick input; require a released and freshly pressed arm button after loss or stop.
- Reject short/nonfinite input without crashing; clamp normalized axes.
- Initially clamp driver speed to +/-2000 ERPM, a previously tested bench range. The m/s conversion remains a provisional reference, not measured vehicle speed.
- Keep servo commands on a preview topic. The current firmware's servo-output support, BEC, neutral, endpoints and wiring are not verified. This preparation alone must not be called steerable ground driving.
- Back up Jetson source files before replacement; preserve unrelated PC and Jetson changes.

Alternatives: a new direct-serial gamepad program would duplicate the ROS stack; retaining the RC receiver needs the transmitter and a different control path. Reusing the existing ROS node and standard VESC driver fits the user's requested gamepad/Jetson workflow.

Acceptance: synthetic-input tests pass in a hardware-isolated ROS domain, native ARM builds succeed, VESC read-only telemetry identifies controller 117, DualSense input mapping is observed, and input-loss behavior is tested without hardware commands. Actual steering and driving are separate observed hardware checks.
