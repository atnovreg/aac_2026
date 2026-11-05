import unittest
import numpy as np
from tools.map_review_cloud import pack_cloud

class CloudTest(unittest.TestCase):
    def test_sloped_floor_filter_and_binary_layout(self):
        # z=.1*x-.5; a point above z=0 can still be on the floor.
        p=np.array([[10,0,.5],[0,0,-.2],[0,0,-.5],[np.nan,0,0]],dtype=float)
        original=p.copy()
        q=pack_cloud(p,np.array([.1,0,-.5]),.15)
        self.assertEqual(q.shape,(1,4))
        np.testing.assert_allclose(q[0],[0,0,-.2,.3],atol=1e-6)
        self.assertEqual(q.dtype,np.dtype('<f4'))
        self.assertEqual(len(q.tobytes()),16)
        np.testing.assert_equal(p,original)

if __name__=='__main__':unittest.main()
