"""Exercise actual MCAP writing/reading and unchanged IMU/point payloads."""
import json
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import serialize_message,deserialize_message
from sensor_msgs.msg import Imu,PointCloud2
from test_filtering import cloud
from tools.lidar_filter.__main__ import export,reader_for
from tools.lidar_filter.verify_bag import verify


class BagExportTests(unittest.TestCase):
    def test_filtered_bag_preserves_unrelated_messages_and_exact_retained_records(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=root/'source';run=root/'run';out=root/'out'
            run.mkdir();out.mkdir()
            np.savetxt(run/'trajectory.tum',[[1700000000,0,0,0,0,0,0,1],[1700000001,0,0,0,0,0,0,1]])
            (run/'config.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':{'mapping':{'extrinsic_R':np.eye(3).ravel().tolist(),'extrinsic_T':[0,0,0]}}}}))
            msg=cloud([(.55,.05,.05,77,0,0,1700000000000010000.),(2,0,0,88,0,1,1700000000000020000.),(3,0,0,99,0,2,1700000000000030000.)]+[(4+i,0,0,9+i,0,i%4,1700000000000040000.+i*10000) for i in range(7)]+[(2,0,0,88,0,1,1700000000000020000.)]*2)
            imu=Imu();imu.header.stamp.sec=1700000000;imu.angular_velocity.x=1.2345
            imu_bytes=serialize_message(imu);cloud_bytes=serialize_message(msg)
            writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(source),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
            for idx,name,kind in [(0,'/livox/lidar','PointCloud2'),(1,'/livox/imu','Imu')]:
                writer.create_topic(rosbag2_py.TopicMetadata(id=idx,name=name,type='sensor_msgs/msg/'+kind,serialization_format='cdr'))
            writer.write('/livox/imu',imu_bytes,1700000000000000000)
            writer.write('/livox/lidar',cloud_bytes,1700000000100000000)
            del writer
            (out/'manifest.json').write_text(json.dumps({'source_run':str(run),'source_bag':str(source),'parameters':{'voxel_size':.1,'floor_plane':[0,0,-.7],'height_range':[.15,1.9]}}))
            np.savez(out/'evidence.npz',voxel_keys=np.array([[5,0,0]]),dynamic=np.array([True]))
            export(SimpleNamespace(output=out))
            verify(out)
            self.assertEqual(json.loads((out/'bag_validation.json').read_text())['status'],'passed')
            reader=reader_for(out/'source_filtered');received=[]
            while reader.has_next(): received.append(reader.read_next())
            self.assertEqual(len(received),2)
            self.assertEqual(received[0],('/livox/imu',imu_bytes,1700000000000000000))
            kept=deserialize_message(received[1][1],PointCloud2)
            self.assertEqual(received[1][2],1700000000100000000)
            self.assertEqual(kept.header,msg.header)
            self.assertEqual(bytes(kept.data),bytes(msg.data)[26:])
            # Source is still readable and unchanged.
            reader=reader_for(source);reader.read_next()
            self.assertEqual(reader.read_next()[1],cloud_bytes)

if __name__=='__main__':unittest.main()
