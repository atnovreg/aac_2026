#!/usr/bin/env python3
"""Run each bag in a fresh FAST-LIO2 process, preserving logs and results."""
import argparse
import datetime
import json
import fcntl
import shutil
import math
import os
import platform
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

WS = Path('/ws')
INSTALL = Path(os.environ.get('F1TENTH_INSTALL_BASE', str(WS/'artifacts/f1tenth'/('jazzy-'+platform.machine())/'install')))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', help='Bag directory under /ws/bag, or all')
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--config', default='/ws/src/aac_2026/navigation/fastlio/config/mid360_bag.yaml')
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error('--rate must be positive and finite')
    bags = sorted((WS/'bag').glob('*/metadata.yaml')) if args.bag == 'all' else [WS/'bag'/args.bag/'metadata.yaml']
    if not bags or any(not p.is_file() for p in bags):
        parser.error('Bag metadata not found')
    (WS/"results").mkdir(exist_ok=True)
    with (WS/"results"/".mapping.lock").open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("Another mapping run is active in this workspace")
        for metadata in bags:
            run(metadata.parent, args)


def run(bag, args):
    run_id = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    out = WS/'results'/bag.name/run_id
    out.mkdir(parents=True)
    config = yaml.safe_load(Path(args.config).read_text())
    config['/**']['ros__parameters']['map_file_path'] = str(out/'map.pcd')
    (out/'config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    lidar_count = next(t['message_count'] for t in metadata['topics_with_message_count'] if t['topic_metadata']['name']=='/livox/lidar')
    children = []
    logs = []
    poses = []
    rclpy.init()
    node = Node('offline_mapping_monitor')
    trajectory = (out/'trajectory.tum').open('w')
    trajectory.write('# timestamp tx ty tz qx qy qz qw; frame camera_init -> body\n')
    def odometry(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        values = [stamp,p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError('Non-finite odometry')
        poses.append(values)
        trajectory.write(' '.join(f'{v:.9f}' for v in values)+'\n')
        trajectory.flush()
    subscription = node.create_subscription(Odometry, '/Odometry', odometry, 1000)
    def start(command, name):
        log = (out/f'{name}.log').open('w'); logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(process)
        return process
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.05)
            for process in children[:2]:
                if process.poll() is not None:
                    raise RuntimeError(f'Mapping process exited: {process.returncode}; inspect {out}')
    try:
        mapper = start([str(INSTALL/'fast_lio/lib/fast_lio/fastlio_mapping'),'--ros-args','--params-file',str(out/'config.yaml')], 'fastlio')
        adapter = start([str(INSTALL/'bag_adapter/lib/bag_adapter/pointcloud_to_livox')], 'adapter')
        deadline=time.monotonic()+30
        while node.count_subscribers('/livox/lidar')<1 or node.count_subscribers('/livox/lidar_custom')<1 or node.count_subscribers('/livox/imu')<1 or node.count_publishers('/Odometry')<1:
            spin(0.1)
            if time.monotonic()>deadline:
                raise RuntimeError('Timed out waiting for mapping subscriptions')
        spin(1)
        player=start(['ros2','bag','play',str(bag),'--clock','100','--rate',str(args.rate),'--delay','1','--disable-keyboard-controls'], 'playback')
        print(f'Mapping {bag.name} at {args.rate}x -> {out}',flush=True)
        last_report=time.monotonic()
        while player.poll() is None:
            spin(0.1)
            if time.monotonic()-last_report>30:
                print(f'{bag.name}: {len(poses)}/{lidar_count} odometry samples',flush=True)
                last_report=time.monotonic()
        if player.returncode:
            raise RuntimeError(f'Playback failed: {player.returncode}')
        drain_deadline = time.monotonic() + 30
        stable_since = time.monotonic()
        previous_count = len(poses)
        while time.monotonic() - stable_since < 2:
            spin(0.1)
            if len(poses) != previous_count:
                stable_since = time.monotonic()
                previous_count = len(poses)
            if time.monotonic() > drain_deadline:
                raise RuntimeError("Mapper did not drain within 30 seconds")
        # SIGINT invokes upstream final PCD saving; wait before declaring success.
        os.killpg(mapper.pid,signal.SIGINT)
        mapper.wait(timeout=120)
        if mapper.returncode not in [0,-signal.SIGINT]:
            raise RuntimeError(f'FAST-LIO exited {mapper.returncode}')
        trajectory.flush()
        pcd=out/'map.pcd'
        if len(poses)<max(5,int(lidar_count*0.95)):
            raise RuntimeError(f'Only {len(poses)}/{lidar_count} odometry samples; retry slower')
        if not pcd.exists() or pcd.stat().st_size<1000:
            raise RuntimeError('PCD map missing or empty')
        with pcd.open('rb') as file:
            header=[]
            while True:
                line=file.readline().decode('ascii').strip(); header.append(line)
                if line.startswith('DATA '): break
                if not line: raise RuntimeError('Invalid PCD header')
        points=int(next(l.split()[1] for l in header if l.startswith('POINTS ')))
        distance=sum(math.dist(a[1:4],b[1:4]) for a,b in zip(poses,poses[1:]))
        summary={'bag':bag.name,'rate':args.rate,'input_lidar_messages':lidar_count,'odometry_samples':len(poses),'map_points':points,'map_bytes':pcd.stat().st_size,'trajectory_length_m':distance,'end_to_start_m':math.dist(poses[0][1:4],poses[-1][1:4]),'position_min_m':[min(p[i] for p in poses) for i in [1,2,3]],'position_max_m':[max(p[i] for p in poses) for i in [1,2,3]],'status':'completed'}
        (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(summary),flush=True)
    except BaseException as e:
        (out/'failure.txt').write_text(str(e)+'\n')
        raise
    finally:
        for process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try: process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL); process.wait()
        for source in (WS/"src/aac_2026/navigation/fastlio/FAST_LIO/Log").glob("*.txt"):
            shutil.copy2(source, out/source.name)
        trajectory.close()
        for log in logs: log.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
