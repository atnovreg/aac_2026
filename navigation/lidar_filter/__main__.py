"""Build temporal free-space evidence, partition a map, and export a filtered bag."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
from system.f1tenth.workspace import find_workspace, resolve_data_path
import shutil
import time

import numpy as np
import yaml
from scipy.spatial import cKDTree
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2

from planning.track_pipeline.projection import read_pcd, xyz
from .filtering import records, partition_message, mapper_effective_timestamps
from .registration import PoseTrajectory
from .temporal import TemporalVoxelEvidence
from .pcd import split_pcd

ROOT=find_workspace(__file__)
SOURCE_RUN=ROOT/'results/lidar_final_20260909_161208/20260910T112720950598Z'
SOURCE_BAG=ROOT/'bag/lidar_final_20260909_161208'


def reader_for(path):
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    return reader


def project_message(msg, trajectory, offset, stride=1):
    points=records(msg)[::stride]
    coordinates=np.c_[points['x'],points['y'],points['z']]
    world,origin,valid=trajectory.project(coordinates,points['timestamp']/1e9,offset)
    # Match Livox valid returns accepted by the mapper; never cast missing/invalid rays.
    ranges=np.linalg.norm(coordinates,axis=1)
    valid &= (ranges>.5)&(points['line']<4)&np.isin(points['tag'] & 0x30,[0,0x10])
    return points,world,origin,valid


def load_context(run):
    poses=np.loadtxt(run/'trajectory.tum',ndmin=2)
    params=yaml.safe_load((run/'config.yaml').read_text())['/**']['ros__parameters']['mapping']
    if not np.allclose(np.array(params['extrinsic_R']).reshape(3,3),np.eye(3)):
        raise ValueError('This registration path requires the recorded identity LiDAR-to-IMU rotation')
    return PoseTrajectory(poses),np.array(params['extrinsic_T'])


def build(args):
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    trajectory,offset=load_context(args.source_run)
    validation=json.loads(args.floor_validation.read_text())
    declared=Path(validation['source_run'])
    if not declared.is_absolute(): declared=ROOT/declared
    if declared.resolve()!=args.source_run.resolve():
        raise ValueError('Floor validation does not belong to the source mapping run')
    plane=validation['floor_plane']
    params=dict(voxel_size=.1,floor_plane=plane,height_range=(.15,1.9),endpoint_margin=.25,
                min_free_frames=args.min_free_frames,min_free_ratio=args.min_free_ratio,min_frame_gap=.5,hit_radius=.15,
                min_sensor_range=.5,min_static_frames=args.min_static_frames or None)
    manifest={'source_run':str(args.source_run.resolve()),'source_bag':str(args.source_bag.resolve()),
              'parameters':params,'point_stride':3,'protect_near_high_wall_m':.20,'scope':'Temporal visibility candidates, not semantic human labels',
              'raw_bag_modified':False,'map_pose_refined':False}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    source=read_pcd(args.source_run/'map.pcd')
    candidates=xyz(source[::3])
    heights=candidates[:,2]-candidates[:,:2]@np.array(plane[:2])-plane[2]
    high_wall=candidates[(heights>2)&(heights<2.5),:2]
    if not len(high_wall): raise ValueError('No high-wall reference for static protection')
    wall_tree=cKDTree(np.unique(np.floor(high_wall/.05),axis=0)*.05+.025)
    # Focus evidence allocation on the recorded route surroundings; distant map points stay intact.
    lo=trajectory.poses[:,1:3].min(axis=0)-5;hi=trajectory.poses[:,1:3].max(axis=0)+5
    candidates=candidates[((candidates[:,:2]>=lo)&(candidates[:,:2]<=hi)).all(axis=1)]
    evidence=TemporalVoxelEvidence(candidates,**params)
    del candidates
    print(f'Candidate voxels: {len(evidence.voxel_keys)}',flush=True)
    reader=reader_for(args.source_bag);last=-np.inf;report=time.monotonic()
    while reader.has_next():
        topic,data,_=reader.read_next()
        if topic!='/livox/lidar':continue
        msg=deserialize_message(data,PointCloud2)
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp-last<.5-1e-7:continue
        points,world,origins,valid=project_message(msg,trajectory,offset,3)
        if not valid.any():continue
        evidence.observe(origins[valid],world[valid],stamp);last=stamp
        if time.monotonic()-report>15:
            print(f'Visibility: {evidence.frames} frames, {stamp-trajectory.poses[0,0]:.1f}s',flush=True);report=time.monotonic()
    dynamic=evidence.finalize()
    protected=wall_tree.query(evidence.voxel_centers[:,:2])[0]<=.20
    dynamic &= ~protected
    np.savez_compressed(out/'evidence.npz',voxel_keys=evidence.voxel_keys,free_counts=evidence.free_counts,
                        hit_counts=evidence.hit_counts,dynamic=dynamic,protected_wall=protected)
    selected=SavedMask(out)
    removed=np.zeros(len(source),dtype=bool)
    for start in range(0,len(source),300000):
        removed[start:start+300000]=selected.mask(xyz(source[start:start+300000]))
    split_pcd(args.source_run/'map.pcd',out/'map_cleaned.pcd',out/'map_removed.pcd',removed)
    shutil.copy2(args.source_run/'trajectory.tum',out/'trajectory.tum')
    summary=evidence.summary()|{'removed_voxels':int(dynamic.sum()),'protected_wall_voxels':int(protected.sum()),'original_map_points':len(source),'removed_map_points':int(removed.sum()),
                'kept_map_points':int((~removed).sum()),'status':'map_partitioned'}
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary),flush=True)


class SavedMask:
    def __init__(self, output):
        self.manifest=json.loads((output/'manifest.json').read_text())
        evidence=np.load(output/'evidence.npz')
        params=self.manifest['parameters'];self.voxel=params['voxel_size'];self.plane=np.array(params['floor_plane']);self.height=params['height_range']
        keys=evidence['voxel_keys'];dynamic=evidence['dynamic']
        self.low=keys.min(axis=0) if len(keys) else np.zeros(3,dtype=np.int64)
        self.shape=keys.max(axis=0)-self.low+1 if len(keys) else np.ones(3,dtype=np.int64)
        self.grid=np.zeros(np.prod(self.shape),dtype=bool)
        self.grid[self.flat(keys[dynamic])]=True

    def flat(self,keys):
        q=keys-self.low
        return (q[:,0]*self.shape[1]+q[:,1])*self.shape[2]+q[:,2]

    def mask(self,world):
        mask=np.zeros(len(world),dtype=bool)
        height=world[:,2]-world[:,:2]@self.plane[:2]-self.plane[2]
        valid=np.isfinite(world).all(axis=1)&(height>=self.height[0])&(height<=self.height[1])
        ids=np.flatnonzero(valid);keys=np.floor(world[valid]/self.voxel).astype(np.int64)
        q=keys-self.low;inside=((q>=0)&(q<self.shape)).all(axis=1)
        mask[ids[inside]]=self.grid[self.flat(keys[inside])]
        return mask


def export(args):
    out=args.output.resolve();mask=SavedMask(out)
    source_bag=resolve_data_path(ROOT,mask.manifest['source_bag']);source_run=resolve_data_path(ROOT,mask.manifest['source_run'])
    trajectory,offset=load_context(source_run)
    reader=reader_for(source_bag)
    bag_path=out/(source_bag.name+'_filtered')
    if bag_path.exists(): raise FileExistsError(bag_path)
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag_path),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    for topic in reader.get_all_topics_and_types(): writer.create_topic(topic)
    stats={'lidar_frames':0,'original_points':0,'removed_points':0,'other_messages':0,'max_tail_time_loss_ns':0,'outside_pose_coverage_points':0,'max_mapper_endpoint_change_ns':0,'minimum_mapper_points':1000000000}
    source_checks=hashlib.sha256();report=time.monotonic();audit=[]
    while reader.has_next():
        topic,data,record_time=reader.read_next()
        if topic=='/livox/lidar':
            msg=deserialize_message(data,PointCloud2)
            points,world,origins,valid=project_message(msg,trajectory,offset)
            removed=mask.mask(world)&valid
            kept,_=partition_message(msg,removed)
            if kept.width<2: raise ValueError('Filtering left too few points in a frame; inspect output before mapping')
            expected=points[~removed].tobytes()
            if bytes(kept.data)!=expected: raise AssertionError('Point records were modified')
            before_times=mapper_effective_timestamps(points)
            after_times=mapper_effective_timestamps(points[~removed])
            if len(after_times)<2 or not len(before_times):
                raise ValueError('Too few FAST-LIO accepted points after filtering')
            mapper_change=int(abs(before_times[-1]-after_times[-1]))
            if mapper_change>5_000_000: raise ValueError('Mapper scan endpoint moved by over 5ms')
            stats['max_mapper_endpoint_change_ns']=max(stats['max_mapper_endpoint_change_ns'],mapper_change)
            stats['minimum_mapper_points']=min(stats['minimum_mapper_points'],len(after_times))
            tail_loss=int(points['timestamp'].max()-points['timestamp'][~removed].max())
            if tail_loss>5_000_000: raise ValueError('Filtering removed over 5ms of frame tail; scan timing needs review')
            stats['max_tail_time_loss_ns']=max(stats['max_tail_time_loss_ns'],tail_loss)
            stats['lidar_frames']+=1;stats['original_points']+=len(points);stats['removed_points']+=int(removed.sum())
            stats['outside_pose_coverage_points']+=int((~np.isfinite(world).all(axis=1)).sum())
            data=serialize_message(kept)
            audit.append((record_time,len(points),int(removed.sum()),tail_loss))
        else:
            stats['other_messages']+=1
            source_checks.update(topic.encode()+str(record_time).encode()+bytes(data))
        writer.write(topic,data,record_time)
        if time.monotonic()-report>15:
            print(f'Export: {stats["lidar_frames"]} frames, {stats["removed_points"]} points excluded',flush=True);report=time.monotonic()
    del writer
    stats['filtered_bag']=str(bag_path);stats['non_lidar_source_sha256']=source_checks.hexdigest()
    stats['retained_point_bytes_preserved']=True;stats['status']='filtered_bag_exported'
    np.savetxt(out/'frame_audit.csv',np.array(audit,dtype=np.int64),delimiter=',',header='record_time_ns,input_points,removed_points,tail_time_loss_ns',comments='',fmt='%d')
    (out/'bag_summary.json').write_text(json.dumps(stats,indent=2)+'\n')
    print(json.dumps(stats),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',choices=['build','export'])
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--min-free-frames',type=int,default=3)
    parser.add_argument('--min-free-ratio',type=float,default=.8)
    parser.add_argument('--min-static-frames',type=int,default=10,help='0 disables the absolute hit-count veto')
    parser.add_argument('--floor-validation',type=Path,default=ROOT/'tracks/indoor_final/validation.json')
    parser.add_argument('--source-run',type=Path,default=SOURCE_RUN)
    parser.add_argument('--source-bag',type=Path,default=SOURCE_BAG)
    args=parser.parse_args()
    (build if args.stage=='build' else export)(args)

if __name__=='__main__':main()
