import unittest
import numpy as np
from planning.geonji.build_geonji_track import box_clearance
class ClearanceTest(unittest.TestCase):
 def test_cell_edge_not_center_distance_and_outer_edge(self):
  mask=np.ones((5,5),bool);mask[2,3]=False
  points=np.array([[2.5,2.5],[2.9,2.5],[.1,.5],[3.5,2.5]])
  np.testing.assert_allclose(box_clearance(mask,points,1.,np.zeros(2)),[.5,.1,.1,0],atol=1e-12)
