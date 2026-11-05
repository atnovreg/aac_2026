"""Publish review masks at fitted road height; independent of saved cloud viewer."""
import sys,json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

def main():
    data=Path(sys.argv[1]);d=np.load(data);res=float(d['resolution']);plane=d['plane'];origin=d['origin']
    prefix=sys.argv[2].rstrip('/') if len(sys.argv)>2 else '/review/area'
    rclpy.init();n=Node('geonji_drivable_area_'+prefix.rsplit('/',1)[-1]);q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    broadcaster=StaticTransformBroadcaster(n);t=TransformStamped();t.header.frame_id='review_world';t.child_frame_id='review_map';t.transform.rotation.w=1.;t.header.stamp=n.get_clock().now().to_msg();broadcaster.sendTransform(t)
    pubs=[];counts={}
    masks={'candidate':d['candidate'],'obstacles':d['obstacle'],'unknown':d['occupancy']==-1,'main_component':d['main_component']}
    for name in ('observed_floor','assumed_floor'):
        if name in d:masks[name]=d[name]
    if (data.parent/'vehicle_clearance.npz').exists():masks['vehicle_centers']=np.load(data.parent/'vehicle_clearance.npz')['vehicle_centers']
    for name,mask in masks.items():
        yy,xx=np.nonzero(mask);x=origin[0]+(xx+.5)*res;y=origin[1]+(yy+.5)*res;z=x*plane[0]+y*plane[1]+plane[2]+.04
        a=np.ascontiguousarray(np.c_[x,y,z],dtype='<f4');msg=PointCloud2();msg.header.frame_id='review_map';msg.header.stamp=n.get_clock().now().to_msg();msg.height=1;msg.width=len(a);msg.point_step=12;msg.row_step=12*len(a);msg.is_dense=True
        msg.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate('xyz')];msg.data=a.tobytes();topic=prefix+'/'+name
        pub=n.create_publisher(PointCloud2,topic,q);pub.publish(msg);pubs.append(pub);counts[topic]=len(a)
    n.get_logger().info(json.dumps(counts))
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
