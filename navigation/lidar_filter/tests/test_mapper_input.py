import unittest
import numpy as np
from test_filtering import cloud
from tools.lidar_filter.filtering import records,mapper_effective_timestamps


class MapperInputTests(unittest.TestCase):
    def test_invalid_tail_does_not_hide_loss_of_last_usable_return(self):
        rows=[(2,0,0,7,0,0,1000.+i) for i in range(10)]
        rows.extend([(2,0,0,7,48,0,1020.),(.1,0,0,7,0,0,1030.)])
        points=records(cloud(rows))
        np.testing.assert_array_equal(mapper_effective_timestamps(points),[1003,1006,1009])
        np.testing.assert_array_equal(mapper_effective_timestamps(points[np.arange(len(points))!=9]),[1003,1006])

if __name__=='__main__':unittest.main()
