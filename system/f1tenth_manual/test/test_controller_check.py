#!/usr/bin/env python3
"""Feed synthetic telemetry to the real preflight in a hardware-free domain."""
import json
import os
import subprocess
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from vesc_msgs.msg import VescStateStamped


class ControllerCheck(unittest.TestCase):
    def test_accepts_idle_drive_channel_and_rejects_unsafe_states(self):
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '177':
            raise RuntimeError('Requires isolated localhost domain 177')
        rclpy.init()
        node = rclpy.create_node('preflight_test_source')
        pub = node.create_publisher(VescStateStamped, '/vesc/sensors/core', 10)
        executable = os.path.join(get_package_prefix('f1tenth_manual'), 'lib',
                                  'f1tenth_manual', 'check_controller.py')
        cases = [({}, True), ({'controller_id': 0}, False),
                 ({'voltage_input': 10.0}, False), ({'fault_code': 2}, False),
                 ({'speed': 2000.0}, False), ({'temp_fet': 75.0}, False),
                 ({'current_motor': float('nan')}, False)]
        try:
            for changes, accepted in cases:
                with self.subTest(changes=changes):
                    msg = VescStateStamped()
                    msg.state.controller_id = 117
                    msg.state.voltage_input = 11.5
                    msg.state.temp_fet = 25.0
                    for key, value in changes.items():
                        setattr(msg.state, key, value)
                    child = subprocess.Popen([executable], stdout=subprocess.PIPE,
                                             stderr=subprocess.PIPE, text=True)
                    try:
                        deadline = time.monotonic() + 10
                        while child.poll() is None and time.monotonic() < deadline:
                            pub.publish(msg)
                            rclpy.spin_once(node, timeout_sec=0.02)
                        output, errors = child.communicate(timeout=2)
                        self.assertEqual(child.returncode == 0, accepted, errors + output)
                        self.assertEqual(json.loads(output)['ok'], accepted)
                    finally:
                        if child.poll() is None:
                            child.kill()
                            child.wait()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main(verbosity=2)
