import unittest
import numpy as np
from planning.geonji.drivable_area import classify

class MaskTest(unittest.TestCase):
    def test_unknown_and_obstacle_are_not_filled(self):
        floor=np.full((9,9),5);obs=np.zeros((9,9),int);static=np.zeros((9,9),bool)
        floor[4,4]=0;obs[2,2]=4;static[6,6]=True
        r=classify(floor,obs,static,.1,obstacle_source="raw_or_static")
        self.assertEqual(r['occupancy'][4,4],-1)
        self.assertEqual(r['occupancy'][2,2],100)
        self.assertEqual(r['occupancy'][6,6],100)
        self.assertEqual(r['occupancy'][1,1],0)
        self.assertFalse(r['candidate'][4,4])
        self.assertLessEqual(r['clearance'][0,0],.1+1e-6)
        self.assertEqual(r['clearance'][4,4],0)

    def test_static_only_ignores_raw_obstacles_without_filling_unknown(self):
        floor=np.full((5,5),5);raw=np.full((5,5),100);static=np.zeros((5,5),bool)
        floor[0,0]=0;static[2,2]=True
        r=classify(floor,raw,static,.2,obstacle_source='static_map')
        self.assertEqual(r['occupancy'][1,1],0)
        self.assertEqual(r['occupancy'][0,0],-1)
        self.assertEqual(r['occupancy'][2,2],100)
        np.testing.assert_array_equal(r['obstacle'],static)

if __name__=='__main__':unittest.main()
