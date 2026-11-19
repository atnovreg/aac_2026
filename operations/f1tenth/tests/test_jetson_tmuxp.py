"""Exercise the real entry points with filesystem fixtures; never contact hardware."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

AAC = Path(__file__).resolve().parents[3]
RUNNER = AAC / 'operations/tmuxp_config/jetson.sh'


class JetsonTmuxpTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='aac tmuxp ')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.live = self.root / 'live_indoor'
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        self.calls = self.root / 'calls.jsonl'
        self.settings = self.root / 'jetson.env'
        self.settings.write_text('')
        self.env = dict(os.environ, PATH=str(self.bin)+os.pathsep+os.environ['PATH'],
                        AAC_SETTINGS=str(self.settings), AAC_WORKSPACE=str(self.root),
                        AAC_LIVE_DIR=str(self.live), AAC_ROS_SETUP=str(self.root/'ros.bash'),
                        AAC_INITIAL_BASE_POSE='[0, 0, 0, 0, 0, 0]',
                        AAC_RUN_MODE='prepare', CALL_LOG=str(self.calls),
                        AAC_RUN_DIR=str(self.root/'run'), STOP_WAIT_SECONDS='0')
        for key in ['AAC_CONTROL_START', 'AAC_VEHICLE_START', 'TMUX', 'TMUX_PANE']:
            self.env.pop(key, None)
        for path in ['ros.bash', 'install/lidar/setup.bash', 'install/manual/setup.bash',
                     'live_indoor/install/local_setup.bash', 'live_indoor/msg-install/local_setup.bash',
                     'live_indoor/map/map.pcd', 'live_indoor/map/map_evidence.npz', 'live_indoor/map/occupancy.yaml',
                     'live_indoor/track/track.csv', 'live_indoor/track/validation.json',
                     'live_indoor/track/soft_profile.npz', 'live_indoor/fastdds-peer.xml',
                     'src/aac_2026/system/f1tenth_manual/config/manual.car.yaml']:
            target = self.root / path
            target.parent.mkdir(parents=True, exist_ok=True)
            target.touch()
        (self.live/'acados/lib').mkdir(parents=True)
        self.program = ('#!' + sys.executable + '\n'
                        'import json,os,sys\n'
                        'from pathlib import Path\n'
                        'name=Path(sys.argv[0]).name\n'
                        'with open(os.environ["CALL_LOG"],"a") as f:\n'
                        ' f.write(json.dumps(dict(name=name,args=sys.argv[1:],'
                        'domain=os.environ.get("ROS_DOMAIN_ID"),'
                        'mode=os.environ.get("AAC_RUN_MODE"),'
                        'run=os.environ.get("AAC_RUN_DIR")))+"\\n")\n'
                        'if name=="tmux":\n'
                        ' if sys.argv[1]=="has-session": sys.exit(int(os.environ.get("NO_SESSION","1")))\n'
                        ' if sys.argv[1]=="list-panes":\n'
                        '  print("%1 0" if "pane_id" in sys.argv[-1] else os.environ.get("PANE_DEAD","1"))\n')
        for name in ['tmux', 'tmuxp', 'ros2', 'pkill']:
            self.make_program(self.bin/name)
        self.make_program(self.live/'acados-venv/bin/python')
        (self.bin/'pgrep').write_text('#!/bin/sh\nexit 1\n')
        (self.bin/'pgrep').chmod(0o755)

    def make_program(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.program)
        path.chmod(0o755)
        return path

    def run_script(self, path, *args, **env):
        return subprocess.run(['bash', str(path), *args], cwd='/tmp',
                              env=dict(self.env, **env), text=True, capture_output=True)

    def read_calls(self):
        return [json.loads(line) for line in self.calls.read_text().splitlines()] if self.calls.exists() else []

    def test_autonomy_refuses_missing_output_before_launching_or_creating_run(self):
        result = self.run_script(AAC/'start.sh')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('AAC_CONTROL_START', result.stdout+result.stderr)
        self.assertIn('AAC_VEHICLE_START', result.stdout+result.stderr)
        self.assertFalse(any(c['name']=='tmuxp' for c in self.read_calls()))
        self.assertFalse((self.live/'runs').exists())

    def test_prepare_check_rejects_empty_nonfinite_and_wrong_shape_seed(self):
        for pose in ['', '[0,0,0]', '[0,0,0,0,0,NaN]', '[true,0,0,0,0,0]']:
            with self.subTest(pose=pose):
                result = self.run_script(AAC/'start.sh', 'check', 'prepare', AAC_INITIAL_BASE_POSE=pose)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('AAC_INITIAL_BASE_POSE', result.stdout+result.stderr)
        self.assertFalse(self.calls.exists())

    def test_check_does_not_start_nodes_or_create_recording(self):
        result = self.run_script(AAC/'start.sh', 'check', 'prepare')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(any(c['name'] in ['ros2', 'tmuxp'] for c in self.read_calls()))
        self.assertFalse((self.live/'runs').exists())

    def test_auto_initialization_launches_without_a_numeric_seed(self):
        (self.root/'run').mkdir()
        result = self.run_script(RUNNER, 'localization', AAC_INITIAL_BASE_POSE='auto')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('initial_base_pose:=auto', self.read_calls()[0]['args'])

    def test_prepare_uses_live_config_and_new_recording_directory(self):
        result = self.run_script(AAC/'start.sh', 'prepare', '-d')
        self.assertEqual(result.returncode, 0, result.stderr)
        call = next(c for c in self.read_calls() if c['name']=='tmuxp')
        self.assertEqual(call['args'], ['load', '-d', str(AAC/'operations/tmuxp_config/config.yaml')])
        self.assertEqual(call['mode'], 'prepare')
        self.assertTrue(Path(call['run']).is_dir())
        self.assertEqual(Path(call['run']).parent, self.live/'runs')

    def test_extra_tmuxp_arguments_cannot_change_the_managed_session(self):
        result = self.run_script(AAC/'start.sh', 'prepare', '-d', '-s', 'other')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(c['name']=='tmuxp' for c in self.read_calls()))
        self.assertFalse((self.live/'runs').exists())

    def test_existing_driver_prevents_duplicate_start(self):
        (self.bin/'pgrep').write_text('#!/bin/sh\nexit 0\n')
        result = self.run_script(AAC/'start.sh', 'prepare', '-d')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Existing vehicle/sensor/localization', result.stderr)
        self.assertFalse(any(c['name']=='tmuxp' for c in self.read_calls()))
        self.assertFalse((self.live/'runs').exists())

    def test_simulation_requires_no_jetson_assets(self):
        result = self.run_script(AAC/'auto.sh', 'sim', '-d', AAC_ROS_SETUP='/missing')
        self.assertEqual(result.returncode, 0, result.stderr)
        call = next(c for c in self.read_calls() if c['name']=='tmuxp')
        self.assertEqual(call['args'], ['load', '-d', str(AAC/'operations/tmuxp_config/simulation.yaml')])

    def test_components_keep_live_clock_and_existing_manual_domain_separation(self):
        (self.root/'run').mkdir()
        for component in ['lidar', 'localization', 'controller', 'vehicle']:
            result = self.run_script(RUNNER, component)
            self.assertEqual(result.returncode, 0, result.stderr)
        calls = [c for c in self.read_calls() if c['name'] in ['ros2', 'python']]
        self.assertEqual([c['domain'] for c in calls], ['43', '43', '43', '42'])
        self.assertIn('use_sim_time:=false', calls[1]['args'])
        self.assertIn('initial_base_pose:=[0.0,0.0,0.0,0.0,0.0,0.0]', calls[1]['args'])
        self.assertIn('control.acados_mpcc.ros_shadow', calls[2]['args'])
        self.assertIn('stage:=drive', calls[3]['args'])
        self.assertIn('config_file:='+str(self.root/'src/aac_2026/system/f1tenth_manual/config/manual.car.yaml'), calls[3]['args'])

    def test_autonomy_commands_receive_the_same_domain_as_localization(self):
        self.env['AAC_CONTROL_START'] = str(self.make_program(self.root/'control start'))
        self.env['AAC_VEHICLE_START'] = str(self.make_program(self.root/'vehicle start'))
        (self.root/'run').mkdir()
        for component in ['controller', 'vehicle']:
            result = self.run_script(RUNNER, component, AAC_RUN_MODE='autonomy')
            self.assertEqual(result.returncode, 0, result.stderr)
        calls = [c for c in self.read_calls() if c['name'].endswith(' start')]
        self.assertEqual([c['domain'] for c in calls], ['43', '43'])

    def test_builtin_autonomy_defaults_to_preview_commands(self):
        (self.root/'run').mkdir()
        manual_path = self.root/'src/aac_2026/system/f1tenth_manual/config/manual.car.yaml'
        manual_path.write_text("""
/vesc/vesc_driver:
  ros__parameters: {speed_min: -10000.0, speed_max: 10000.0, servo_min: 0.2, servo_max: 0.8}
/vesc/ackermann_to_vesc:
  ros__parameters: {speed_to_erpm_gain: -4614.0, speed_to_erpm_offset: 0.0,
                    steering_angle_to_servo_gain: 1.0, steering_angle_to_servo_offset: 0.46}
/manual_control:
  ros__parameters: {arm_button: 2, stop_button: 1, deadman_button: 5}
""")
        env = {'AAC_RUN_MODE': 'autonomy',
               'AAC_AUTONOMY_PROFILE': str(AAC/'system/f1tenth/config/agx/indoor_drive.yaml'),
               'AAC_AUTONOMY_HARDWARE': '0'}
        for component in ['vehicle', 'controller']:
            result = self.run_script(RUNNER, component, **env)
            self.assertEqual(result.returncode, 0, result.stderr)
        vehicle, controller = self.read_calls()
        self.assertIn('hardware:=false', vehicle['args'])
        self.assertIn('drive_speed_limit:=2.2', controller['args'])
        self.assertEqual([c['domain'] for c in (vehicle, controller)], ['43', '43'])

    @unittest.skipUnless(shutil.which('tmux') and shutil.which('tmuxp'), 'tmux and tmuxp required')
    def test_real_tmuxp_keeps_settings_with_an_existing_server_and_finalizes_bags(self):
        # All ROS/vehicle binaries remain fakes; only the isolated tmux server is real.
        real_tmux = shutil.which('tmux')
        (self.bin/'tmux').unlink()
        (self.bin/'tmuxp').unlink()
        socket_dir = self.root/'sockets'
        socket_dir.mkdir()
        self.env['TMUX_TMPDIR'] = str(socket_dir)
        clean_env = {k:v for k,v in self.env.items() if not k.startswith('AAC_')}
        subprocess.run([real_tmux, 'new-session', '-d', '-s', 'unrelated', 'sleep 60'],
                       env=clean_env, check=True, capture_output=True, text=True)
        self.addCleanup(lambda: subprocess.run([real_tmux, 'kill-server'], env=clean_env,
                                               capture_output=True))
        # Server environment predates AAC overrides. Settings must come from this start.
        self.settings.write_text('export CALL_LOG='+repr(str(self.calls))+'\n')
        process = self.program + (
            'import signal,time\n'
            'def finish(signum,frame):\n'
            ' if sys.argv[1:3]==["bag","record"]:\n'
            '  path=Path(sys.argv[sys.argv.index("-o")+1]); path.mkdir()\n'
            '  (path/"metadata.yaml").write_text("finalized\\n")\n'
            ' sys.exit(0)\n'
            'signal.signal(signal.SIGINT,finish)\n'
            'while True: time.sleep(.1)\n')
        for path in [self.bin/'ros2', self.live/'acados-venv/bin/python']:
            path.write_text(process)
        result = self.run_script(AAC/'start.sh', 'prepare', '-d')
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        deadline = time.monotonic()+10
        calls = []
        while time.monotonic() < deadline:
            calls = [c for c in self.read_calls() if c['name'] in ['ros2','python']]
            if len(calls)==7:
                break
            time.sleep(.1)
        self.assertEqual(len(calls), 7, result.stdout+result.stderr)
        runs = {c['run'] for c in calls}
        self.assertEqual(len(runs), 1)
        run = Path(runs.pop())
        self.assertTrue((run/'settings.env').is_file())
        result = self.run_script(AAC/'stop.sh', STOP_WAIT_SECONDS='10')
        self.assertEqual(result.returncode, 0, result.stderr)
        for name in ['sensors','ps5']:
            self.assertEqual((run/name/'metadata.yaml').read_text(), 'finalized\n')
        # Stopping AAC must leave the other session alone.
        self.assertEqual(subprocess.run([real_tmux, 'has-session', '-t', '=unrelated'],
                                       env=clean_env).returncode, 0)

    def test_stop_only_closes_requested_session_after_processes_exit(self):
        result = self.run_script(AAC/'stop.sh', 'AAC_TEST', NO_SESSION='0')
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self.read_calls()
        self.assertFalse(any(c['name']=='pkill' for c in calls))
        kills = [c['args'] for c in calls if c['args'][0]=='kill-session']
        self.assertEqual(kills, [['kill-session', '-t', '=AAC_TEST']])

    def test_stop_does_not_force_kill_unfinished_recorder(self):
        result = self.run_script(AAC/'stop.sh', 'AAC_TEST', NO_SESSION='0', PANE_DEAD='0')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(c['args'][0]=='kill-session' for c in self.read_calls()))


if __name__ == '__main__':
    unittest.main()
