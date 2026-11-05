"""The filter must remove only the sensor-relative region and retain exact records."""
import copy
import unittest
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from tools.lidar_filter.filtering import filter_message, validate_boxes


def cloud(rows, padding=0):
    # Independent recorded wire layout; last 8 bytes are absolute epoch-ns float64.
    dtype=np.dtype({'names':['x','y','z','intensity','tag','line','timestamp'],
        'formats':['<f4']*4+['u1','u1','<f8'], 'offsets':[0,4,8,12,16,17,18], 'itemsize':26})
    points=np.array(rows,dtype=dtype)
    msg=PointCloud2();msg.header.frame_id='livox_frame';msg.header.stamp.sec=1700000000
    msg.height=1;msg.width=len(points);msg.point_step=26;msg.row_step=26*len(points)+padding
    msg.fields=[PointField(name=n,offset=o,datatype=d,count=1) for n,o,d in zip(dtype.names,[0,4,8,12,16,17,18],[7,7,7,7,2,2,8])]
    msg.data=points.tobytes()+b'X'*padding;msg.is_dense=True
    return msg


class FilteringTests(unittest.TestCase):
    def test_excludes_only_box_preserves_timestamp_intensity_and_input(self):
        rows=[(-.5,0,0,7,16,1,1700000000000000256.), (2,0,0,37,16,2,1700000000000010496.),
              (-.5,1,0,63,0,3,1700000000000020480.), (-.5,0,-1,11,16,0,1700000000000030464.)]
        msg=cloud(rows,padding=8);original=copy.deepcopy(msg)
        kept,removed=filter_message(msg,[{'min':[-1,-.4,-.3],'max':[0,.4,.5]}])
        self.assertEqual(msg,original)
        self.assertEqual(kept.width,3);self.assertEqual(removed.width,1)
        self.assertEqual(bytes(kept.data),bytes(original.data)[26:104])
        self.assertEqual(bytes(removed.data),bytes(original.data)[:26])
        self.assertEqual(kept.header,original.header);self.assertEqual(kept.fields,original.fields)
        self.assertEqual(kept.row_step,78)

    def test_multiple_boxes_boundary_and_far_wall(self):
        msg=cloud([(0,0,0,1,0,0,1), (2,0,0,2,0,0,2), (10,0,0,3,0,0,3)])
        kept,removed=filter_message(msg,[{'min':[-1,-1,-1],'max':[0,1,1]}, {'min':[1,-1,-1],'max':[3,1,1]}])
        self.assertEqual(kept.width,1);self.assertEqual(removed.width,2)
        self.assertEqual(bytes(kept.data),bytes(msg.data)[52:78])

    def test_rejects_bad_boxes_and_bad_layout(self):
        for boxes in [[],[{'min':[2,0,0],'max':[1,1,1]}],[{'min':[0,0,float('nan')],'max':[1,1,1]}]]:
            with self.assertRaises(ValueError): validate_boxes(boxes)
        msg=cloud([(1,1,1,1,0,0,1)]);msg.fields[-1].offset=17
        with self.assertRaises(ValueError): filter_message(msg,[{'min':[0,0,0],'max':[2,2,2]}])

    def test_all_removed_is_represented_as_empty_frame(self):
        msg=cloud([(0,0,0,1,0,0,1)])
        kept,removed=filter_message(msg,[{'min':[-1,-1,-1],'max':[1,1,1]}])
        self.assertEqual(kept.width,0);self.assertEqual(len(kept.data),0)
        self.assertEqual(removed.width,1)

if __name__=='__main__':unittest.main()
