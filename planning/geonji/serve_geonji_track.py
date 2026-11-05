"""RViz track limits and optional offline simulated trajectory (no actuation)."""
import argparse,json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point

def main():
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path);args=p.parse_args()
 track_path=Path(json.loads((args.run/'provenance.json').read_text())['track']) if args.run else Path('tracks/geonji_083146_assumed')
 # Host provenance paths become workspace paths inside Docker.
 if not track_path.exists():track_path=Path('/ws/tracks')/track_path.name
 d=np.load(track_path/'visualization.npz');plane=d['plane']
 rclpy.init();node=Node('geonji_track_review');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE);pub=node.create_publisher(MarkerArray,'/review/geonji_track',q);arr=MarkerArray()
 curves=[('Centerline',d['center'],(.15,.35,1.),True),('Left center limit',d['left'],(.1,.8,.25),True),('Right center limit',d['right'],(1.,.1,.65),True)]
 if args.run:
  sim=np.genfromtxt(args.run/'trajectory.csv',delimiter=',',names=True);curves.append(('Simulated vehicle',np.c_[sim['x'],sim['y']],(1.,.85,.1),False))
 for i,(name,xy,color,closed) in enumerate(curves):
  if closed:xy=np.vstack([xy,xy[0]])
  m=Marker();m.header.frame_id='review_map';m.header.stamp=node.get_clock().now().to_msg();m.ns=name;m.id=i;m.type=Marker.LINE_STRIP;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=.10;m.color.r,m.color.g,m.color.b=color;m.color.a=1.
  m.points=[Point(x=float(x),y=float(y),z=float(x*plane[0]+y*plane[1]+plane[2]+.15+i*.02)) for x,y in xy];arr.markers.append(m)
 pub.publish(arr);node.get_logger().info(f'Published {len(curves)} track/simulation curves')
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
