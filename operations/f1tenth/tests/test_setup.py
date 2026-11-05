"""Setup must reconstruct aliases without overwriting user files."""
import importlib.util
from pathlib import Path
import tempfile
import unittest


def setup_module():
    source = Path(__file__).resolve().parents[1] / 'setup_workspace.py'
    spec = importlib.util.spec_from_file_location('aac_setup', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SetupTests(unittest.TestCase):
    def test_links_are_relative_and_setup_is_repeatable(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / 'src/module.py'
            source.parent.mkdir()
            source.write_text('preserved')
            alias = root / 'tools/module.py'
            setup = setup_module()
            setup.ensure_links({alias: source})
            setup.ensure_links({alias: source})
            self.assertEqual(alias.read_text(), 'preserved')
            self.assertFalse(alias.readlink().is_absolute())

    def test_collision_is_detected_before_any_link_is_created(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / 'source'
            source.write_text('source')
            existing = root / 'existing'
            existing.write_text('user data')
            new = root / 'new'
            with self.assertRaises(FileExistsError):
                setup_module().ensure_links({new: source, existing: source})
            self.assertFalse(new.exists())
            self.assertEqual(existing.read_text(), 'user data')

    def test_missing_target_is_rejected_before_any_link_is_created(self):
        setup = setup_module()
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / 'source'
            source.touch()
            with self.assertRaises(FileNotFoundError):
                setup.ensure_links({root/'first': source, root/'second': root/'missing'})
            self.assertFalse((root/'first').exists())
