#!/usr/bin/env python3
"""Integration check against an actual recorded message (run in Docker)."""
import signal
import subprocess
import platform
import time
import numpy as np
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg

reader=rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri='/ws/bag/lidar_2forward_1reverse_20260909_160207',storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
while reader.has_next():
    topic,data,_=reader.read_next()
    if topic=='/livox/lidar':
        original=deserialize_message(data,PointCloud2)
        break
rclpy.init()
node=rclpy.create_node('adapter_integration_test')
received=[]
pub=node.create_publisher(PointCloud2,'/livox/lidar',10)
sub=node.create_subscription(CustomMsg,'/livox/lidar_custom',received.append,10)
process=subprocess.Popen(['/ws/artifacts/f1tenth/jazzy-'+platform.machine()+'/install/bag_adapter/lib/bag_adapter/pointcloud_to_livox'])
try:
    end=time.monotonic()+10
    while pub.get_subscription_count()<1 or node.count_publishers('/livox/lidar_custom')<1:
        rclpy.spin_once(node,timeout_sec=.05)
        assert time.monotonic()<end,'Adapter discovery timed out'
    pub.publish(original)
    while not received:
        rclpy.spin_once(node,timeout_sec=.05)
        assert time.monotonic()<end,'Adapter did not publish'
    output=received[0]
    assert output.point_num==original.width*original.height
    assert output.header==original.header
    base=original.header.stamp.sec*10**9+original.header.stamp.nanosec
    assert output.timebase==base
    dtype=np.dtype({'names':['x','y','z','intensity','tag','line','timestamp'],'formats':['<f4','<f4','<f4','<f4','u1','u1','<f8'],'offsets':[0,4,8,12,16,17,18],'itemsize':26})
    points=np.frombuffer(original.data,dtype=dtype)
    order=np.argsort(points['timestamp'],kind='stable')
    for actual,expected in zip(output.points,points[order]):
        assert actual.x==expected['x'] and actual.y==expected['y'] and actual.z==expected['z']
        assert actual.tag==expected['tag'] and actual.line==expected['line']
        assert actual.reflectivity==int(expected['intensity'])
        assert abs((base+actual.offset_time)-int(expected['timestamp']))<=256
    original.fields[-1].name='missing_timestamp'
    pub.publish(original)
    assert process.wait(timeout=10)==1,'Malformed schema must fail closed'
    print(f'PASS: {output.point_num} real points preserved; malformed schema rejected')
finally:
    if process.poll() is None:
        process.send_signal(signal.SIGINT); process.wait(timeout=10)
    node.destroy_node(); rclpy.shutdown()
