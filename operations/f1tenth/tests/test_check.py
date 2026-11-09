"""Current layout checks must work independently of one-time migration records."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

AAC = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('layout_check', AAC/'operations/f1tenth/verify_layout.py')
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


class CheckTests(unittest.TestCase):
    def source_tree(self, root):
        aac = root/'src/aac_2026'
        for name in check.REQUIRED_FILES:
            path = aac/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('# editable source\n')
        (root/'.f1tenth-workspace').touch()
        return aac

    def test_source_check_allows_development_without_old_migration_files(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            aac = self.source_tree(root)
            (aac/'control/acados_mpcc/controller.py').write_text('# changed during development\n')
            self.assertEqual(check.check_layout(aac, source_only=True), [])
            self.assertFalse((aac/'operations/f1tenth/import_manifest.json').exists())

    def test_missing_entry_point_and_broken_data_link_are_reported(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            aac = self.source_tree(root)
            (aac/'operations/f1tenth/run.sh').unlink()
            (root/'bag').symlink_to('missing-recordings')
            for name in ['tracks', 'results']:
                (root/name).mkdir()
            errors = '\n'.join(check.check_layout(aac))
            self.assertIn('operations/f1tenth/run.sh', errors)
            self.assertIn('bag', errors)


if __name__ == '__main__':
    unittest.main()
