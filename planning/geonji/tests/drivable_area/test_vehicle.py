import unittest
import numpy as np
from planning.geonji.drivable_area import classify
from planning.geonji.vehicle_area_clearance import fit_centers
class VehicleTest(unittest.TestCase):
    def test_narrow_observed_strip_is_not_enough_for_vehicle(self):
        floor=np.zeros((30,30),int);floor[2:28,13:17]=3
        e=classify(floor,np.zeros_like(floor),np.zeros_like(floor,bool),.1)
        c,r=fit_centers(e['candidate'],e['clearance'],.56,.19,.05)
        self.assertFalse(c.any());self.assertGreater(r,.34)
        d,r2=fit_centers(e['candidate'],e['clearance'],.19,.56,.05)
        np.testing.assert_array_equal(c,d);self.assertEqual(r,r2)
