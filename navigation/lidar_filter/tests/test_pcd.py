import tempfile
import unittest
from pathlib import Path
import numpy as np
from navigation.lidar_filter.pcd import split_pcd
from planning.track_pipeline.projection import read_pcd


class PcdTests(unittest.TestCase):
    def test_partition_preserves_all_fields_and_original_file(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);source=folder/'source.pcd'
            header=b'VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA binary\n'
            payload=np.array([[1,2,3,77],[4,5,6,88],[7,8,9,99]],dtype='<f4').tobytes()
            source.write_bytes(header+payload)
            split_pcd(source,folder/'kept.pcd',folder/'removed.pcd',np.array([False,True,False]))
            self.assertEqual(source.read_bytes(),header+payload)
            self.assertEqual(read_pcd(folder/'kept.pcd').tobytes(),payload[:16]+payload[32:])
            self.assertEqual(read_pcd(folder/'removed.pcd').tobytes(),payload[16:32])

if __name__=='__main__':unittest.main()
