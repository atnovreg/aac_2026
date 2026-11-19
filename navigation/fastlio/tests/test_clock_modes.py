"""Run with ROS Jazzy and the built fast_lio/RViz packages; no sensors needed."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.parameter_client import AsyncParameterClient
import yaml


AAC = Path(__file__).resolve().parents[3]


class MappingClockTests(unittest.TestCase):
    def test_mapping_and_rviz_follow_environment_and_explicit_override(self):
        # Catch a missing environment default or an RViz left on wall time.
        for index, (setting, override, expected) in enumerate([
            ('true', None, True), ('false', None, False),
            ('true', 'false', False), ('false', 'true', True),
            (None, None, False),
        ]):
            with self.subTest(environment=setting, override=override):
                self.check_clock(setting, override, expected, 80 + index)

    def check_clock(self, setting, override, expected, domain):
        with tempfile.TemporaryDirectory() as directory:
            config = yaml.safe_load((AAC/'navigation/fastlio/config/mid360_bag.yaml').read_text())
            config['/**']['ros__parameters']['pcd_save']['pcd_save_en'] = False
            Path(directory, 'clock.yaml').write_text(yaml.safe_dump(config))
            env = dict(os.environ, ROS_LOG_DIR=directory,
                       LIBGL_ALWAYS_SOFTWARE='1',
                       ROS_DOMAIN_ID=str(domain))
            if setting is None:
                env.pop('USE_SIM_TIME', None)
            else:
                env['USE_SIM_TIME'] = setting
            command = [
                'ros2', 'launch', str(AAC/'navigation/fastlio/FAST_LIO/launch/mapping.launch.py'),
                f'config_path:={directory}', 'config_file:=clock.yaml', 'rviz:=true',
            ]
            if override is not None:
                command.append(f'use_sim_time:={override}')
            with open(Path(directory, 'launch.log'), 'w+') as log:
                process = subprocess.Popen(command, cwd=directory, env=env,
                                           stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                node = None
                try:
                    rclpy.init(domain_id=domain)
                    node = rclpy.create_node('mapping_clock_test')
                    for name in ['laser_mapping', 'rviz']:
                        client = AsyncParameterClient(node, name)
                        self.assertTrue(client.wait_for_services(timeout_sec=15), name)
                        # Service discovery can precede discovery of the response endpoint.
                        deadline = time.monotonic() + 15
                        while time.monotonic() < deadline:
                            future = client.get_parameters(['use_sim_time'])
                            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
                            if future.done():
                                break
                            future.cancel()
                        self.assertTrue(future.done(), name)
                        self.assertFalse(future.cancelled(), name)
                        self.assertEqual(future.result().values[0].bool_value, expected, name)

                except BaseException:
                    log.flush()
                    log.seek(0)
                    print(log.read())
                    raise
                finally:
                    if node is not None:
                        node.destroy_node()
                    if rclpy.ok():
                        rclpy.shutdown()
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=5)
                    if process.returncode not in (0, -signal.SIGINT):
                        log.seek(0)
                        print(log.read())


if __name__ == '__main__':
    unittest.main()
