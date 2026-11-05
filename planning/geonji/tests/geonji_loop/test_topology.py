import unittest
import numpy as np
from tools.check_geonji_loop import encloses
class LoopTest(unittest.TestCase):
 def test_ring_and_real_gap(self):
  m=np.zeros((9,9),bool);m[2,2:7]=True;m[6,2:7]=True;m[2:7,2]=True;m[2:7,6]=True
  self.assertTrue(encloses(m,(4,4)))
  m[2,4]=False
  self.assertFalse(encloses(m,(4,4)))
 def test_unrelated_loop_does_not_count(self):
  m=np.zeros((12,12),bool);m[1,1:5]=True;m[4,1:5]=True;m[1:5,1]=True;m[1:5,4]=True
  self.assertFalse(encloses(m,(8,8)))
if __name__=='__main__':unittest.main()
