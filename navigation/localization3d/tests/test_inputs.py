"""One immutable input bundle serves simulation and preparation."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import prepare
from navigation.localization3d import geometry


class InputTests(unittest.TestCase):
    def test_bundled_inputs_match_the_original_experiment_hashes(self):
        manifest = json.loads((prepare.INPUTS/'manifest.json').read_text())
        for name, digest in manifest['input_sha256'].items():
            self.assertEqual(hashlib.sha256((prepare.INPUTS/name).read_bytes()).hexdigest(), digest, name)
        geometry.analytic_check()

    def test_preparation_reuses_shared_inputs_without_overwriting_existing_data(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root/'map.pcd'
            source.write_text('test source')
            plane = root/'results/건지광장/map_edit/20261001_similarity_obstacles/drivable_area/area.npz'
            plane.parent.mkdir(parents=True)
            np.savez(plane, plane=np.zeros(3))
            points = np.array([(0., 0., 0.), (.2, .2, .1)],
                              dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
            out = root/'new-inputs'
            with patch.object(prepare, 'WS', root), patch.object(prepare, 'SOURCE', source), \
                    patch.object(prepare, 'read_pcd', return_value=points):
                prepare.main(out)
                for name in ['track.csv', 'validation.json', 'soft_profile.npz',
                             'zones_map_xy.geojson', 'area_layers.npz']:
                    self.assertEqual((out/name).read_bytes(), (prepare.INPUTS/name).read_bytes())
                original = (out/'manifest.json').read_bytes()
                with self.assertRaises(FileExistsError):
                    prepare.main(out)
                self.assertEqual((out/'manifest.json').read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
