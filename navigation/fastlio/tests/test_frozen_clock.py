#!/usr/bin/env python3
"""Mapping must continue processing buffered sensor data while /clock is frozen."""
import signal
import subprocess
import platform
import time
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry

rclpy.init()
node=rclpy.create_node('frozen_clock_test')
lidar=node.create_publisher(PointCloud2,'/livox/lidar',100)
imu=node.create_publisher(Imu,'/livox/imu',100)
poses=[]
sub=node.create_subscription(Odometry,'/Odometry',poses.append,1000)
children=[]
try:
    children.append(subprocess.Popen(['/ws/artifacts/f1tenth/jazzy-'+platform.machine()+'/install/fast_lio/lib/fast_lio/fastlio_mapping','--ros-args','--params-file','/ws/src/aac_2026/navigation/fastlio/config/mid360_bag.yaml','-p','pcd_save.pcd_save_en:=false'],stdout=subprocess.DEVNULL))
    children.append(subprocess.Popen(['/ws/artifacts/f1tenth/jazzy-'+platform.machine()+'/install/bag_adapter/lib/bag_adapter/pointcloud_to_livox'],stdout=subprocess.DEVNULL))
    deadline=time.monotonic()+15
    while lidar.get_subscription_count()<1 or imu.get_subscription_count()<1 or node.count_subscribers('/livox/lidar_custom')<1:
        rclpy.spin_once(node,timeout_sec=.05)
        assert time.monotonic()<deadline,'Discovery failed'
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri='/ws/bag/lidar_2forward_1reverse_20260909_160207',storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    while reader.has_next():
        topic,data,_=reader.read_next()
        if topic=='/livox/lidar':
            cloud=deserialize_message(data,PointCloud2)
            last_lidar_time=cloud.header.stamp.sec+cloud.header.stamp.nanosec*1e-9
            lidar.publish(cloud)
        elif topic=='/livox/imu': imu.publish(deserialize_message(data,Imu))
        rclpy.spin_once(node,timeout_sec=0)
        time.sleep(.001)
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.01)
    assert poses, 'No odometry output while /clock frozen'
    last_pose_time=poses[-1].header.stamp.sec+poses[-1].header.stamp.nanosec*1e-9
    assert last_pose_time>=last_lidar_time-.3, 'Buffered tail scans did not drain'
    print(f'PASS: {len(poses)} poses computed without advancing /clock')
finally:
    for process in reversed(children):
        if process.poll() is None:
            process.send_signal(signal.SIGINT); process.wait(timeout=15)
    node.destroy_node(); rclpy.shutdown()
