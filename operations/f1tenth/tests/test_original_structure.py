"""Keep one data name and import code from its actual AAC category."""
import importlib
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

AAC = Path(__file__).resolve().parents[3]
WS = AAC.parent.parent


class OriginalStructureTests(unittest.TestCase):
    def test_old_saved_bag_paths_use_the_canonical_directory(self):
        from system.f1tenth.workspace import resolve_data_path
        for value in ['bags/run', 'bag/run', '/ws/bags/run', '/ws/bag/run']:
            self.assertEqual(resolve_data_path(WS, value), WS/'bag/run')
        self.assertEqual(resolve_data_path(WS, '/ws/results/run'), WS/'results/run')
        self.assertEqual(resolve_data_path(WS, '/external/recording.bag'),
                         Path('/external/recording.bag'))

    def test_original_bag_name_is_the_only_recording_entry(self):
        self.assertTrue((WS/'bag').is_dir())
        self.assertTrue(any((WS/'bag').rglob('metadata.yaml')))
        self.assertFalse((WS/'bags').exists())
        self.assertFalse((WS/'bags').is_symlink())

    def test_no_parallel_legacy_source_tree_at_workspace_root(self):
        for name in ['tools', 'scripts', 'config', 'requirements', 'compose.yaml']:
            with self.subTest(name=name):
                self.assertFalse((WS/name).exists())

    def test_added_support_files_reuse_existing_aac_categories(self):
        for name in ['common', 'simulation', 'archive', 'docs', 'f1tenth.sh']:
            self.assertFalse((AAC/name).exists(), name)
        for name in ['system/f1tenth/workspace.py',
                     'system/f1tenth_gazebo_replay/generate.py',
                     'operations/f1tenth/archive/COLCON_IGNORE',
                     'operations/f1tenth/docs/F1TENTH_시작하기.md',
                     'operations/f1tenth/run.sh']:
            self.assertTrue((AAC/name).is_file(), name)

    def test_python_modules_use_actual_functional_categories(self):
        for name in ['control.acados_mpcc.controller', 'control.mpcc_tuner.config',
                     'planning.track_pipeline.geometry', 'planning.geonji.build_geonji_track',
                     'navigation.map_review.map_review_cloud',
                     'system.f1tenth_gazebo_replay.generate']:
            module = importlib.import_module(name)
            self.assertTrue(Path(module.__file__).is_relative_to(AAC), name)

    def test_fresh_setup_needs_no_legacy_source_aliases_or_assets(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            target = root/'src/aac_2026/operations/f1tenth'
            target.mkdir(parents=True)
            (target/'setup_workspace.py').write_bytes(
                (AAC/'operations/f1tenth/setup_workspace.py').read_bytes())
            subprocess.run([sys.executable, str(target/'setup_workspace.py')], check=True,
                           capture_output=True, text=True)
            self.assertTrue((root/'bag').is_dir())
            for name in ['tools', 'scripts', 'config', 'requirements', 'bags']:
                self.assertFalse((root/name).exists(), name)


if __name__ == '__main__':
    unittest.main()
