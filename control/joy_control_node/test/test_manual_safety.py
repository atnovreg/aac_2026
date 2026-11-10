#!/usr/bin/env python3
"""Real ROS safety tests; run with ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=177.

No VESC driver is started. Override JOY_CONTROL_EXECUTABLE for an installed
baseline or candidate binary. Tests would catch removal of freshness checks,
manual-only gating, input validation, stop priority, or arm-edge handling.
"""
import collections
import math
import os
import subprocess
import tempfile
import time
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy


class ManualSafety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '177':
            raise RuntimeError('Run only in isolated localhost ROS domain 177')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.log = tempfile.TemporaryFile(mode='w+')
        binary = os.environ.get('JOY_CONTROL_EXECUTABLE')
        binary = binary or os.path.join(get_package_prefix('joy_control_node'),
                                       'lib', 'joy_control_node', 'joy_control_node_exe')
        command = [binary]
        command += ['--ros-args', '-p', 'manual_only:=true', '-p', 'input_timeout:=0.3',
                    '-p', 'maximum_speed:=1.0']
        self.command = command
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT)
        self.node = rclpy.create_node('manual_safety_probe')
        self.samples = collections.deque(maxlen=1000)
        self.publisher = self.node.create_publisher(Joy, '/joy', 10)
        self.auto_publisher = self.node.create_publisher(AckermannDriveStamped, '/ackermann_auto', 10)
        self.subscription = self.node.create_subscription(
            AckermannDriveStamped, '/ackermann_drive',
            lambda msg: self.samples.append((time.monotonic(), msg.drive.speed,
                                             msg.drive.steering_angle)), 10)
        deadline = time.monotonic() + 5
        while self.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertIsNone(self.process.poll(), 'Controller exited before test')
        self.assertGreater(self.publisher.get_subscription_count(), 0, 'No controller subscriber')

    def tearDown(self):
        self.process.terminate()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=3)
        self.node.destroy_node()
        self.log.close()

    @staticmethod
    def joy(arm=0, stop=0, deadman=0, throttle=0.5):
        msg = Joy()
        msg.axes = [0.2, 0.0, 0.0, 0.0, float(throttle), 0.0]
        msg.buttons = [int(stop), int(arm), 0, 0, 0, int(deadman), 0, 0]
        return msg

    def pump(self, duration, message=None, auto=None):
        deadline = time.monotonic() + duration
        next_publish = 0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                if message is not None:
                    message.header.stamp = self.node.get_clock().now().to_msg()
                    self.publisher.publish(message)
                if auto is not None:
                    self.auto_publisher.publish(auto)
                next_publish = now + 0.03
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def armed(self):
        self.pump(0.15, self.joy())
        self.pump(0.25, self.joy(arm=1, deadman=1))
        self.pump(0.25, self.joy(deadman=1))
        self.assertTrue(self.samples, 'No drive messages')
        self.assertAlmostEqual(self.samples[-1][1], 0.5, places=4)

    def assert_stopped(self):
        self.assertTrue(self.samples, 'No stop output')
        self.assertTrue(math.isfinite(self.samples[-1][1]))
        self.assertEqual(self.samples[-1][1], 0.0)
        self.assertEqual(self.samples[-1][2], 0.0)

    def test_stale_joystick_stops_and_requires_rearm(self):
        self.armed()
        self.pump(0.8)
        self.assert_stopped()
        self.pump(0.3, self.joy(deadman=1, throttle=0.8))
        self.assert_stopped()
        self.pump(0.2, self.joy(arm=1, deadman=1))
        self.assertAlmostEqual(self.samples[-1][1], 0.5, places=4)

    def test_deadman_release_does_not_pass_auto_in_manual_mode(self):
        self.armed()
        auto = AckermannDriveStamped()
        auto.drive.speed = 0.8
        self.pump(0.3, self.joy(deadman=0), auto=auto)
        self.assert_stopped()

    def test_stop_wins_over_arm_and_requires_arm_release(self):
        self.pump(0.15, self.joy())
        begin = time.monotonic()
        self.pump(0.3, self.joy(arm=1, stop=1, deadman=1))
        output = [x for x in self.samples if x[0] >= begin + 0.1]
        self.assertTrue(output)
        self.assertTrue(all(x[1] == 0 for x in output), 'Stop lost priority to arm')
        self.pump(0.2, self.joy(arm=1, deadman=1))
        self.assert_stopped()

    def test_short_joystick_stops_without_crashing(self):
        self.armed()
        self.pump(0.3, Joy())
        self.assertIsNone(self.process.poll(), 'Malformed joystick crashed controller')
        self.assert_stopped()

    def test_nonfinite_joystick_stops(self):
        self.armed()
        self.pump(0.3, self.joy(deadman=1, throttle=float('nan')))
        self.assert_stopped()

    def test_axis_is_clamped_to_normalized_range(self):
        self.armed()
        self.pump(0.3, self.joy(deadman=1, throttle=2.5))
        self.assertAlmostEqual(self.samples[-1][1], 1.0, places=4)

    def test_arm_held_at_startup_requires_release(self):
        self.pump(0.35, self.joy(arm=1, deadman=1))
        self.assert_stopped()
        self.pump(0.15, self.joy())
        self.pump(0.25, self.joy(arm=1, deadman=1))
        self.assertAlmostEqual(self.samples[-1][1], 0.5, places=4)

    def test_nonfinite_timeout_is_rejected(self):
        self.process.terminate()
        self.process.wait(timeout=3)
        self.process = subprocess.Popen(
            self.command + ['-p', 'input_timeout:=.nan'],
            stdout=self.log, stderr=subprocess.STDOUT)
        try:
            code = self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.fail('Nonfinite timeout accepted; stale-input guard could be disabled')
        self.assertNotEqual(code, 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
