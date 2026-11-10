#!/usr/bin/env python3
"""Verify the real serial decoder -> ROS temperature path with a pseudo-terminal.

The fake peer only answers telemetry requests. No physical serial port is opened.
"""
import binascii
import os
import pty
import signal
import struct
import subprocess
import tempfile
import time
import tty
import unittest

import rclpy
from ament_index_python.packages import get_package_prefix
from vesc_msgs.msg import VescStateStamped


class DriverTemperature(unittest.TestCase):
    def test_decoded_primary_temperatures_are_published(self):
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '177':
            raise RuntimeError('Requires isolated localhost domain 177')
        master, slave = pty.openpty()
        tty.setraw(slave)
        os.set_blocking(master, False)
        binary = os.path.join(get_package_prefix('vesc_driver'), 'lib',
                              'vesc_driver', 'vesc_driver_node')
        log = tempfile.TemporaryFile()
        child = subprocess.Popen([binary, '--ros-args', '-p', 'port:=' + os.ttyname(slave)],
                                 stdout=log, stderr=subprocess.STDOUT)
        rclpy.init()
        node = rclpy.create_node('driver_temperature_probe')
        samples = []
        sub = node.create_subscription(VescStateStamped, '/sensors/core',
                                       lambda msg: samples.append(msg.state), 10)
        values = bytearray(73)
        values[0] = 4  # COMM_GET_VALUES; primary FET/motor temperature fields.
        struct.pack_into('>hh', values, 1, 750, 310)
        struct.pack_into('>H', values, 27, 115)
        values[58] = 117
        fw = bytes([0, 5, 2]) + b'TEST_DEVICE\0' + bytes(40)
        pending = bytearray()
        try:
            deadline = time.monotonic() + 6
            while not samples and time.monotonic() < deadline:
                try:
                    pending.extend(os.read(master, 4096))
                except BlockingIOError:
                    pass
                while len(pending) >= 5:
                    if pending[0] != 2:
                        del pending[0]
                        continue
                    length = pending[1]
                    if len(pending) < length + 5:
                        break
                    request = bytes(pending[2:2 + length])
                    del pending[:length + 5]
                    response = fw if request[0] == 0 else values if request[0] == 4 else None
                    if response is not None:
                        crc = binascii.crc_hqx(response, 0)
                        os.write(master, bytes([2, len(response)]) + response +
                                 struct.pack('>H', crc) + bytes([3]))
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertTrue(samples, 'No telemetry decoded from test serial peer')
            self.assertEqual(samples[-1].controller_id, 117)
            self.assertEqual(samples[-1].voltage_input, 11.5)
            self.assertEqual(samples[-1].temp_fet, 75.0)
            self.assertEqual(samples[-1].temp_motor, 31.0)
        finally:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
            node.destroy_subscription(sub)
            node.destroy_node()
            rclpy.shutdown()
            os.close(master)
            os.close(slave)
            log.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
