"""Display diagnostic loop and its bottlenecks, never publish controller commands."""
from pathlib import Path
import json,numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point

def main():
 base=Path('results/건지광장/loop_check/083146_v1');d=np.load(base/'reference_loop.npz');rows=json.loads((base/'bottlenecks.json').read_text());xy=d['xy'];plane=d['plane']
 rclpy.init();node=Node('geonji_loop_feasibility');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE);pub=node.create_publisher(MarkerArray,'/review/loop_check',q);arr=MarkerArray()
 def marker(ns,i,kind,color):
  m=Marker();m.header.frame_id='review_map';m.header.stamp=node.get_clock().now().to_msg();m.ns=ns;m.id=i;m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1.;m.color.r,m.color.g,m.color.b=color;m.color.a=1.;return m
 def point(x,y,h=.12):return Point(x=float(x),y=float(y),z=float(x*plane[0]+y*plane[1]+plane[2]+h))
 m=marker('DIAGNOSTIC_ONLY_reference_loop',0,Marker.LINE_STRIP,(.1,.4,1.));m.scale.x=.12;m.points=[point(x,y) for x,y in xy];arr.markers.append(m)
 for i,r in enumerate(rows):
  x,y=r['center_xy_m'];m=marker('bottleneck_labels',i,Marker.TEXT_VIEW_FACING,(1.,1.,0.));m.pose.position=point(x,y,1.2);m.scale.z=.8;m.text=r['id'];arr.markers.append(m)
  m=marker('bottleneck_points',i,Marker.SPHERE,(1.,.1,.7));m.pose.position=point(x,y,.2);m.scale.x=m.scale.y=m.scale.z=.35;arr.markers.append(m)
  a,b=r['start_index'],r['end_index']
  if b>a:
   m=marker('bottleneck_intervals',i,Marker.LINE_STRIP,(1.,.1,.7));m.scale.x=.22;m.points=[point(x,y,.15) for x,y in xy[a:b+1]];arr.markers.append(m)
 pub.publish(arr);node.get_logger().info(f'Diagnostic loop and {len(rows)} bottleneck labels published; NOT a controller path.')
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
