"""Exercise relocated command wrappers without starting containers or Gazebo."""
import json
import os
from pathlib import Path
import platform
import posixpath
import subprocess
import sys
import tempfile
import unittest

AAC = Path(__file__).resolve().parents[3]
WS = AAC.parent.parent


class WrapperTests(unittest.TestCase):
    def test_compose_asset_targets_resolve_workspace_symlinks(self):
        output = subprocess.check_output(
            ['docker', 'compose', '-f', str(AAC/'docker/f1tenth/compose.yaml'),
             'config', '--format', 'json'], cwd=WS, text=True)
        config = json.loads(output)['services']['fastlio']
        mounts = {item['target']: item['source'] for item in config['volumes']}
        for name in ['bags', 'tracks', 'results', 'vendor/acados']:
            with self.subTest(name=name):
                alias = WS/name
                target = posixpath.normpath(posixpath.join(
                    posixpath.dirname('/ws/'+name), os.readlink(alias)))
                self.assertIn(target, mounts)
                self.assertEqual(Path(mounts[target]).resolve(), alias.resolve())

    def test_compose_wrappers_supply_existing_absolute_assets(self):
        with tempfile.TemporaryDirectory() as folder:
            stub = Path(folder)/'docker'
            stub.write_text('#!' + sys.executable + '\nimport os,json\n'
                            'print(json.dumps(dict(assets=os.environ.get("F1TENTH_ASSETS"))))\n')
            stub.chmod(0o755)
            env = dict(os.environ, PATH=folder+os.pathsep+os.environ['PATH'])
            env.pop('F1TENTH_ASSETS', None)
            for script, action in [('navigation/fastlio/scripts/slam.sh', 'shell'),
                                   ('simulation/f1tenth_gazebo_replay/scripts/gazebo_replay.sh', 'build')]:
                with self.subTest(script=script):
                    output = subprocess.check_output(['bash', str(AAC/script), action], env=env, text=True)
                    self.assertEqual(json.loads(output)['assets'], str((WS/'bags').resolve().parent))

    def test_replay_build_test_and_plugin_search_share_artifact_directory(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            commands = root/'commands'
            commands.mkdir()
            log = root/'calls.jsonl'
            code = ('#!' + sys.executable + '\nimport os,sys,json\n'
                    'from pathlib import Path\n'
                    'with open(os.environ["CALL_LOG"],"a") as f:\n'
                    ' f.write(json.dumps(dict(command=Path(sys.argv[0]).name,args=sys.argv[1:],'
                    'plugin=os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH")))+"\\n")\n'
                    'if Path(sys.argv[0]).name=="python3": print("/tmp/test-world.sdf")\n')
            for name in ['cmake', 'ctest', 'python3', 'gz']:
                path = commands/name
                path.write_text(code)
                path.chmod(0o755)
            env = dict(os.environ, PATH=str(commands)+os.pathsep+os.environ['PATH'], CALL_LOG=str(log))
            env.pop('GZ_SIM_SYSTEM_PLUGIN_PATH', None)
            subprocess.run(['bash', str(AAC/'simulation/f1tenth_gazebo_replay/scripts/gazebo_replay_container.sh'),
                            '--headless'], cwd=root, env=env, check=True, capture_output=True)
            calls = [json.loads(line) for line in log.read_text().splitlines()]
            expected = str(root/'artifacts/f1tenth'/('jazzy-'+platform.machine())/'gazebo_replay')
            configure = calls[0]['args']
            self.assertEqual(configure[configure.index('-B')+1], expected)
            test = next(c for c in calls if c['command']=='ctest')['args']
            self.assertEqual(test[test.index('--test-dir')+1], expected)
            gazebo = next(c for c in calls if c['command']=='gz')
            self.assertEqual(gazebo['plugin'].split(os.pathsep)[0], expected)
