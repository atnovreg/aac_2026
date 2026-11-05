"""Relocation checks for the actual consolidated workspace."""
import importlib
from pathlib import Path
import sys
import subprocess
import unittest

AAC = Path(__file__).resolve().parents[3]
WS = AAC.parent.parent
sys.path.insert(0, str(WS))


class ConsolidationTests(unittest.TestCase):
    def test_python_api_uses_aac_sources_and_ros_workspace(self):
        module = importlib.import_module('tools.acados_mpcc.controller')
        self.assertTrue(Path(module.__file__).resolve().is_relative_to(AAC))
        find = importlib.import_module('tools.workspace').find_workspace
        self.assertEqual(find(module.__file__), WS)
        self.assertTrue((WS / 'config/vehicles/slash_58276_74.yaml').is_file())

    def test_livox_has_real_hardware_source_and_messages(self):
        driver = AAC / 'sensors/livox_ros_driver2'
        self.assertTrue((driver / 'src/livox_ros_driver2.cpp').is_file())
        self.assertTrue((driver / 'msg/CustomMsg.msg').is_file())
        self.assertTrue((driver / 'config').is_dir())

    def test_localization_includes_solver_dependencies(self):
        folder = AAC / 'navigation/localization3d'
        for name in ['estimator.py', 'closed_loop.py', 'soft_runtime.py', 'solver_margin.py']:
            self.assertTrue((folder / name).is_file(), name)
        self.assertTrue((folder / 'inputs/map_geometry.npz').is_file())

    def test_localization_imports_without_original_experiment_directory(self):
        folder = AAC / 'navigation/localization3d'
        code = (
            "import closed_loop, audit, display; from pathlib import Path; "
            "assert closed_loop.WS == Path(" + repr(str(WS)) + "); "
            "assert closed_loop.BASE == Path.cwd(); "
            "assert Path(audit.geometry.__file__).parent == Path.cwd()"
        )
        result = subprocess.run([sys.executable, '-W', 'ignore::DeprecationWarning', '-c', code],
                                cwd=folder, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_archive_and_ros1_are_excluded(self):
        self.assertTrue((AAC / 'archive/f1tenth/COLCON_IGNORE').is_file())
        self.assertFalse((AAC / 'navigation/dynamic_lio/upstream').exists())

    def test_data_and_local_environments_are_reachable(self):
        for name in ['bags', 'tracks', 'results', 'vendor/acados', '.venv-acados']:
            self.assertTrue((WS / name).is_dir(), name)
        for folder in [WS / 'tools', WS / 'config']:
            for path in folder.iterdir():
                if path.is_symlink():
                    self.assertTrue(path.exists(), str(path))


if __name__ == '__main__':
    unittest.main()
