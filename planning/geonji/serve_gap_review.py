"""Latched numbered gap markers in the unchanged review_map frame."""
import json,argparse
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--area',default='results/건지광장/drivable_area/083146_v2_20cm');parser.add_argument('--topic',default='/review/gaps');parser.add_argument('--remaining-only',action='store_true');args=parser.parse_args()
 rows=json.loads(Path('results/건지광장/gap_review/083146_v1/regions.json').read_text());d=np.load(Path(args.area)/'area.npz');plane=d['plane'];poses=d['poses']
 vehicle=np.load(Path(args.area)/'vehicle_clearance.npz')['vehicle_centers'];ij=np.floor((poses[:,1:3]-d['origin'])/d['resolution']).astype(int);bad=~vehicle[ij[:,1],ij[:,0]]
 rclpy.init();node=Node('numbered_gap_review_static' if args.remaining_only else 'numbered_gap_review');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE);pub=node.create_publisher(MarkerArray,args.topic,q);array=MarkerArray()
 for row in rows:
  m=Marker();m.header.frame_id='review_map';m.header.stamp=node.get_clock().now().to_msg();m.ns='gap_labels';m.id=row['id'];m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD;m.pose.orientation.w=1.;x,y=row['center_xy'];m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=float(x*plane[0]+y*plane[1]+plane[2]+1.0);m.scale.z=.65;m.color.r=1.;m.color.g=1.;m.color.a=1.;m.text=f'G{row["id"]:02d}';
  if args.remaining_only:
   count=int(bad[row['pose_start']:row['pose_end']+1].sum());m.text+=f' ({count})'
   if count==0:m.color.r=0.;m.color.g=1.;m.color.b=1.
  array.markers.append(m)
  line=Marker();line.header=m.header;line.ns='gap_intervals';line.id=row['id'];line.type=Marker.LINE_STRIP;line.action=Marker.ADD;line.pose.orientation.w=1.;line.scale.x=.16;line.color.a=1.
  if 'obstacle' in row['dominant_reason']:line.color.r=1.;line.color.g=.2
  else:line.color.r=1.;line.color.g=.1;line.color.b=1.
  for p in poses[row['pose_start']:row['pose_end']+1]:line.points.append(Point(x=float(p[1]),y=float(p[2]),z=float(p[1]*plane[0]+p[2]*plane[1]+plane[2]+.12)))
  if args.remaining_only:
   line.type=Marker.LINE_LIST;line.points=[];line.color.r=1.;line.color.g=.1;line.color.b=1.
   for j in range(row['pose_start'],row['pose_end']):
    if bad[j] or bad[j+1]:
     for p in poses[j:j+2]:line.points.append(Point(x=float(p[1]),y=float(p[2]),z=float(p[1]*plane[0]+p[2]*plane[1]+plane[2]+.12)))
  if line.points:array.markers.append(line)
 pub.publish(array);node.get_logger().info(f'{len(rows)} labels and {len(array.markers)-len(rows)} intervals on {args.topic}')
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
