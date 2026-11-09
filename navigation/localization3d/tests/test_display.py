import struct
import tempfile
from pathlib import Path
import unittest
import json
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from display import ring
import display
import xml.etree.ElementTree as ET


class DisplayTests(unittest.TestCase):
    def test_saved_run_builds_a_world_from_shared_display_assets(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);run=root/'run';run.mkdir()
            poses=np.repeat(np.eye(4)[None],2,axis=0);poses[1,0,3]=1.
            np.savez(run/'loop.npz',time=np.array([0.,1.]),gt=poses,odometry=poses,estimate=poses)
            (run/'summary.json').write_text(json.dumps({'completed_lap':True}))
            (root/'results').mkdir()
            (root/'results/audit.json').write_text(json.dumps({'status':'PASS','run':str(run)}))
            with patch.object(display,'HERE',root):
                display.main(SimpleNamespace(run=run))
            output=root/'results/display'
            world=ET.parse(output/'world.sdf').getroot().find('world')
            self.assertEqual(len(world.findall("model[@name='vehicle_gt']")),1)
            for name in ['preferred_region.stl','soft_band.stl','raceline.stl']:
                self.assertEqual((output/name).read_bytes(),(display.ASSETS/name).read_bytes())
            self.assertFalse(any('/experiment/gazebo_equiv100/' in e.text
                                 for e in world.iter('uri')))

    def test_visible_circle_center_is_recorded_xyz_on_a_tilted_pose(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'ring.stl';ring(path,.5)
            with path.open('rb') as f:
                f.read(80);count=struct.unpack('<I',f.read(4))[0]
                faces=np.frombuffer(f.read(),dtype=[('n','<f4',3),('v','<f4',(3,3)),('a','<u2')],count=count)['v']
            local_center=faces.mean(axis=(0,1))
            rotation=Rotation.from_euler('xyz',[.2,-.1,.5]).as_matrix()
            position=np.array([1.,2.,3.])
            np.testing.assert_allclose(rotation@local_center+position,position,atol=1e-7)


if __name__=='__main__':unittest.main()
