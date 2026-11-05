import struct
import tempfile
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from display import ring


class DisplayTests(unittest.TestCase):
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
