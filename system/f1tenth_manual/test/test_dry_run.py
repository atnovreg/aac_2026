#!/usr/bin/env python3
"""Exercise the installed launch/controller/converter; never starts a driver."""
import collections
import os
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class DryRun(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '177':
            raise RuntimeError('Requires isolated localhost domain 177')
        rclpy.init()
        cls.log = tempfile.TemporaryFile()
        cls.pc_profile = os.environ.get('MANUAL_TEST_GAMEPAD_PROFILE', '')
        command = ['ros2', 'launch', 'f1tenth_manual', 'manual.launch.py',
                   'stage:=dry-run', 'gamepad:=false']
        if cls.pc_profile:
            command.append('gamepad_profile:=' + cls.pc_profile)
        cls.process = subprocess.Popen(
            command, start_new_session=True,
            stdout=cls.log, stderr=subprocess.STDOUT)
        cls.node = rclpy.create_node('manual_chain_test')
        cls.pub = cls.node.create_publisher(Joy, '/manual/joy', 10)
        cls.erpm = collections.deque(maxlen=1000)
        cls.servo = collections.deque(maxlen=1000)
        cls.subs = [
            cls.node.create_subscription(Float64, '/manual/erpm_preview',
                                         lambda msg: cls.erpm.append(msg.data), 10),
            cls.node.create_subscription(Float64, '/manual/servo_preview',
                                         lambda msg: cls.servo.append(msg.data), 10)]
        deadline = time.monotonic() + 8
        while cls.pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.05)

    @classmethod
    def tearDownClass(cls):
        os.killpg(cls.process.pid, signal.SIGINT)
        try:
            cls.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(cls.process.pid, signal.SIGKILL)
            cls.process.wait(timeout=3)
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.log.close()

    def pump(self, duration, arm=0, stop=0, deadman=0, throttle=0.5, publish=True):
        deadline = time.monotonic() + duration
        next_publish = 0
        while time.monotonic() < deadline:
            if publish and time.monotonic() >= next_publish:
                msg = Joy()
                if self.pc_profile:
                    msg.axes = [0.2, 0.0, 1.0, 0.0, float(throttle), 1.0, 0.0, 0.0]
                    msg.buttons = [stop, arm, 0, 0, 0, deadman, 0, 0, 0, 0, 0, 0, 0]
                else:
                    msg.axes = [0.2, 0.0, 0.0, 1.0, 1.0, float(throttle), 0.0, 0.0]
                    msg.buttons = [0, stop, arm, 0, 0, deadman, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                self.pub.publish(msg)
                next_publish = time.monotonic() + 0.03
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def setUp(self):
        self.assertIsNone(self.process.poll())
        self.assertGreater(self.pub.get_subscription_count(), 0)
        self.pump(0.25, stop=1)
        self.pump(0.2)
        self.assertTrue(self.erpm)
        self.assertEqual(self.erpm[-1], 0.0)

    def test_no_hardware_route(self):
        names = self.node.get_node_names_and_namespaces()
        self.assertFalse(any('vesc_driver' in name for name, _ in names))
        topics = dict(self.node.get_topic_names_and_types())
        self.assertNotIn('/manual/motor_speed', topics)
        self.assertNotIn('/vesc/commands/servo/position', topics)

    def test_mapping_deadman_stop_and_limit(self):
        self.pump(0.25, arm=1, deadman=1)
        self.assertAlmostEqual(self.erpm[-1], 692.1, delta=0.05)
        self.assertAlmostEqual(self.servo[-1], 0.53, delta=0.001)
        self.pump(0.2, deadman=1, throttle=2.5)
        self.assertAlmostEqual(self.erpm[-1], 1384.2, delta=0.05)
        self.pump(0.2)
        self.assertEqual(self.erpm[-1], 0.0)
        self.pump(0.2, deadman=1)
        self.pump(0.2, stop=1, arm=1, deadman=1)
        self.assertEqual(self.erpm[-1], 0.0)

    def test_stale_input_disarms_full_chain(self):
        self.pump(0.25, arm=1, deadman=1)
        self.assertGreater(self.erpm[-1], 0)
        self.pump(0.7, publish=False)
        self.assertEqual(self.erpm[-1], 0.0)
        self.pump(0.3, deadman=1)
        self.assertEqual(self.erpm[-1], 0.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
