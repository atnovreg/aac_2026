"""Exercise relocated command wrappers without starting containers or Gazebo."""
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import unittest

AAC = Path(__file__).resolve().parents[3]
WS = AAC.parent.parent


class WrapperTests(unittest.TestCase):
    def test_compose_mounts_canonical_data_paths_without_creating_old_names(self):
        output = subprocess.check_output(
            ['docker', 'compose', '-f', str(AAC/'docker/f1tenth/compose.yaml'),
             'config', '--format', 'json'], cwd=WS, text=True)
        config = json.loads(output)['services']['fastlio']
        mounts = {item['target']: item['source'] for item in config['volumes']}
        for name in ['bag', 'tracks', 'results']:
            with self.subTest(name=name):
                alias = WS/name
                self.assertEqual(Path(mounts['/ws/'+name]).resolve(), alias.resolve())
                volume = next(v for v in config['volumes'] if v['target']=='/ws/'+name)
                self.assertFalse(volume['bind']['create_host_path'])

    def test_compose_fresh_workspace_does_not_reference_bags(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for name in ['bag', 'tracks', 'results']:
                (root/name).mkdir()
            env = dict(os.environ, F1TENTH_WS=str(root))
            output = subprocess.check_output(
                ['docker', 'compose', '-f', str(AAC/'docker/f1tenth/compose.yaml'),
                 'config', '--format', 'json'], env=env, cwd=root, text=True)
            volumes = json.loads(output)['services']['fastlio']['volumes']
            for name in ['bag', 'tracks', 'results']:
                volume = next(v for v in volumes if v['target']=='/ws/'+name)
                self.assertEqual(Path(volume['source']), root/name)
            self.assertFalse((root/'bags').exists())

    def test_compose_wrappers_supply_the_canonical_workspace(self):
        with tempfile.TemporaryDirectory() as folder:
            stub = Path(folder)/'docker'
            stub.write_text('#!' + sys.executable + '\nimport os,json\n'
                            'print(json.dumps(dict(workspace=os.environ.get("F1TENTH_WS"))))\n')
            stub.chmod(0o755)
            env = dict(os.environ, PATH=folder+os.pathsep+os.environ['PATH'])
            env.pop('F1TENTH_ASSETS', None)
            for script, action in [('navigation/fastlio/scripts/slam.sh', 'shell'),
                                   ('system/f1tenth_gazebo_replay/scripts/gazebo_replay.sh', 'build')]:
                with self.subTest(script=script):
                    output = subprocess.check_output(['bash', str(AAC/script), action], env=env, text=True)
                    self.assertEqual(json.loads(output)['workspace'], str(WS))

    def test_rviz_container_explicitly_mounts_linked_data(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            script = root/'src/aac_2026/navigation/map_review/scripts/rviz_map_review.sh'
            script.parent.mkdir(parents=True)
            script.write_bytes((AAC/'navigation/map_review/scripts/rviz_map_review.sh').read_bytes())
            (root/'.f1tenth-workspace').touch()
            data = root/'results/건지광장/rviz_review/083146/clouds.npz'
            data.parent.mkdir(parents=True)
            data.touch()
            auth = root/'auth'
            auth.touch()
            commands = root/'commands'
            commands.mkdir()
            stub = commands/'docker'
            stub.write_text('#!' + sys.executable + '\nimport sys,json\n'
                            'if sys.argv[1:3]==["container","inspect"]: sys.exit(1)\n'
                            'print(json.dumps(sys.argv[1:]))\n')
            stub.chmod(0o755)
            env = dict(os.environ, PATH=str(commands)+os.pathsep+os.environ['PATH'],
                       XAUTHORITY=str(auth))
            output = subprocess.check_output(['bash', str(script)], env=env, text=True)
            args = json.loads(output)
            for name in ['bag', 'tracks', 'results']:
                self.assertIn(f'{root/name}:/ws/{name}:ro', args)

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
            subprocess.run(['bash', str(AAC/'system/f1tenth_gazebo_replay/scripts/gazebo_replay_container.sh'),
                            '--headless'], cwd=root, env=env, check=True, capture_output=True)
            calls = [json.loads(line) for line in log.read_text().splitlines()]
            expected = str(root/'artifacts/f1tenth'/('jazzy-'+platform.machine())/'gazebo_replay')
            configure = calls[0]['args']
            self.assertEqual(configure[configure.index('-B')+1], expected)
            test = next(c for c in calls if c['command']=='ctest')['args']
            self.assertEqual(test[test.index('--test-dir')+1], expected)
            gazebo = next(c for c in calls if c['command']=='gz')
            self.assertEqual(gazebo['plugin'].split(os.pathsep)[0], expected)
