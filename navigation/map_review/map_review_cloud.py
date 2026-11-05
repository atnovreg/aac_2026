"""Prepare immutable saved-map review data; serve it with ROS2 in the existing image."""
import argparse
import json
from pathlib import Path
import numpy as np
from system.f1tenth.workspace import find_workspace, resolve_data_path


def pack_cloud(xyz,plane,minimum_height=None):
    p=np.asarray(xyz);p=p[np.isfinite(p).all(axis=1)]
    height=p[:,2]-p[:,:2]@plane[:2]-plane[2]
    keep=np.ones(len(p),bool) if minimum_height is None else height>minimum_height
    return np.ascontiguousarray(np.column_stack([p[keep],height[keep]]),dtype='<f4')


def prepare(run,out,metrics):
    from scipy.spatial.transform import Rotation,Slerp
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores,get_typestore
    from sensors.bag_adapter.pointcloud_layout import OUTPUT_DTYPE
    from planning.track_pipeline.projection import read_pcd
    poses=np.loadtxt(run/'trajectory.tum');times=poses[:,0]-poses[0,0]
    m=json.loads(metrics.read_text());plane=np.array(m['ground_plane_z_ax_by_c']);lo,hi=np.array(m['roi_xy'])
    def crop(p):return p[(p[:,:2]>=lo).all(1)&(p[:,:2]<=hi).all(1)&np.isfinite(p).all(1)]
    def voxel(p):return np.unique(np.floor(p/.1).astype(np.int32),axis=0)
    pc=read_pcd(run/'map_static.pcd');static=crop(np.column_stack([pc[k] for k in 'xyz']))
    static=(voxel(static).astype(float)+.5)*.1
    rot=Slerp(times,Rotation.from_quat(poses[:,4:8]));store=get_typestore(Stores.ROS1_NOETIC)
    source=resolve_data_path(find_workspace(__file__), json.loads((run/'manifest.json').read_text())['bag'])
    blocks=[];batch=[];scans=0
    with Reader(source) as reader:
        cs=[c for c in reader.connections if c.msgtype.endswith('PointCloud2')]
        for conn,_,data in reader.messages(connections=cs):
            msg=store.deserialize_ros1(data,conn.msgtype);p=np.frombuffer(msg.data,dtype=OUTPUT_DTYPE)
            t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9-poses[0,0]+p['time'].astype(float)
            valid=(t>=0)&(t<=times[-1]);t=t[valid];p=p[valid]
            if not len(t):continue
            local=np.column_stack([p[k] for k in 'xyz']).astype(float)+[-.011,-.02329,.04412]
            world=rot(t).apply(local)+np.column_stack([np.interp(t,times,poses[:,i]) for i in (1,2,3)])
            batch.append(crop(world));scans+=1
            if scans%100==0:blocks.append(voxel(np.concatenate(batch)));batch=[]
    if batch:blocks.append(voxel(np.concatenate(batch)))
    raw=(np.unique(np.concatenate(blocks),axis=0).astype(float)+.5)*.1
    out.mkdir(parents=True,exist_ok=True)
    np.savez(out/'clouds.npz',input_map=raw.astype('<f4'),static_map=static.astype('<f4'),plane=plane,poses=poses)
    info={'source_run':str(run),'review_metrics':str(metrics),'frame':'review_map','voxel_m':.1,'roi_xy':m['roi_xy'],'input_scans':scans,'input_points':len(raw),'static_points':len(static),'floor_hide_threshold_m':.15,'note':'All scans within saved pose times; input after bag conversion, before angular projection. Cropped to trajectory XY bounds +5m. No live localization. Intensity field means height above fitted plane.'}
    (out/'manifest.json').write_text(json.dumps(info,ensure_ascii=False,indent=2)+'\n');print(json.dumps(info),flush=True)


def serve(data):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2,PointField
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    rclpy.init();node=Node('saved_map_review')
    # Display-only identity anchor; this is not a measured vehicle transform.
    anchor=TransformStamped();anchor.header.frame_id='review_world';anchor.child_frame_id='review_map'
    anchor.header.stamp=node.get_clock().now().to_msg();anchor.transform.rotation.w=1.0
    broadcaster=StaticTransformBroadcaster(node);broadcaster.sendTransform(anchor)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    d=np.load(data);pubs=[];messages=[];counts={}
    for key in ('input_map','static_map'):
        for suffix,threshold in [('',None),('_no_floor',.15)]:
            a=pack_cloud(d[key],d['plane'],threshold);msg=PointCloud2()
            msg.header.frame_id='review_map';msg.header.stamp=node.get_clock().now().to_msg()
            msg.height=1;msg.width=len(a);msg.is_dense=True;msg.is_bigendian=False
            msg.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(['x','y','z','intensity'])]
            msg.point_step=16;msg.row_step=16*len(a);msg.data=a.tobytes()
            topic='/review/'+key+suffix;pubs.append(node.create_publisher(PointCloud2,topic,qos));messages.append(msg);counts[topic]=len(a)
    path=RosPath();path.header.frame_id='review_map';path.header.stamp=node.get_clock().now().to_msg()
    for row in d['poses']:
        p=PoseStamped();p.header.frame_id='review_map';sec=int(row[0]);p.header.stamp.sec=sec;p.header.stamp.nanosec=int((row[0]-sec)*1e9)
        p.pose.position.x,p.pose.position.y,p.pose.position.z=map(float,row[1:4]);p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w=map(float,row[4:8]);path.poses.append(p)
    pubs.append(node.create_publisher(RosPath,'/review/trajectory',qos));messages.append(path)
    for pub,msg in zip(pubs,messages):pub.publish(msg)
    node.get_logger().info(json.dumps({'point_counts':counts,'trajectory_poses':len(path.poses),'ready':True}))
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    p=argparse.ArgumentParser();sub=p.add_subparsers(dest='command',required=True)
    a=sub.add_parser('prepare');a.add_argument('--run',type=Path,required=True);a.add_argument('--output',type=Path,required=True);a.add_argument('--metrics',type=Path,required=True)
    a=sub.add_parser('serve');a.add_argument('data',type=Path)
    args=p.parse_args()
    if args.command=='prepare':prepare(args.run,args.output,args.metrics)
    else:serve(args.data)
