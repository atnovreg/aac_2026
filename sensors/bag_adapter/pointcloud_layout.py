"""Point layout used by historical converted PointCloud2 recordings.

Kept identical to the previous Dynamic-LIO adapter's output layout so
map-review tools can read existing recordings without importing its ROS1 stack.
"""
import numpy as np

OUTPUT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                        ('intensity', '<f4'), ('time', '<f4'), ('ring', '<u2')])
