#!/usr/bin/env python3
"""Read-only bench preflight. Requires the telemetry launch; sends no commands."""
import json
import math
import time

import rclpy
from vesc_msgs.msg import VescStateStamped


def main():
    rclpy.init()
    node = rclpy.create_node('manual_controller_check')
    samples = []
    subscription = node.create_subscription(
        VescStateStamped, '/vesc/sensors/core', lambda msg: samples.append(msg.state), 10)
    deadline = time.monotonic() + 8.0
    while len(samples) < 25 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    error = None
    if len(samples) < 25:
        error = 'No reliable VESC telemetry (need 25 samples)'
    else:
        for state in samples:
            if state.controller_id != 117:
                error = 'Wrong USB channel: expected drive controller 117'
            elif state.fault_code != 0:
                error = 'VESC reports a fault'
            elif not all(math.isfinite(x) for x in (
                    state.voltage_input, state.speed, state.current_motor, state.temp_fet)):
                error = 'Nonfinite VESC telemetry'
            elif not 10.2 <= state.voltage_input <= 12.65:
                error = '3S battery outside bench-start range 10.2..12.65 V'
            elif abs(state.speed) > 20 or abs(state.current_motor) > 0.5:
                error = 'Motor is not idle'
            elif state.temp_fet >= 70:
                error = 'VESC is too hot'
            if error:
                break
    result = {'ok': error is None, 'samples': len(samples), 'error': error}
    if samples:
        last = samples[-1]
        result.update(controller_id=last.controller_id, voltage=last.voltage_input,
                      erpm=last.speed, motor_current=last.current_motor,
                      fault=last.fault_code, temp_fet=last.temp_fet)
    print(json.dumps(result, ensure_ascii=False))
    node.destroy_subscription(subscription)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if error is None else 1


if __name__ == '__main__':
    raise SystemExit(main())
