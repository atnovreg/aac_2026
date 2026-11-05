import unittest
import numpy as np
from planning.geonji.build_geonji_track import route_search_mask
class RouteChoiceTest(unittest.TestCase):
 def test_gate_only_removes_search_cells(self):
  original=np.ones((40,40),bool);original[0,0]=False
  chosen=route_search_mask(original,np.array([-36.,0.]),.5,True)
  self.assertTrue(np.all(chosen<=original))
  self.assertTrue(original[25,2])
  self.assertFalse(chosen[25,2])
  self.assertTrue(chosen[25,17])
