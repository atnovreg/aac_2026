import argparse
from pathlib import Path
from system.f1tenth.workspace import find_workspace
import json
import yaml
import numpy as np
from .projection import read_pcd,project
from .geometry import extract_loop,resample_closed,ray_boundaries,sample_grid,self_intersections,track_limits,validate_strip


def main():
    parser=argparse.ArgumentParser(description='Extract a structural indoor MPCC corridor from FAST-LIO output')
    parser.add_argument('--config',default='src/aac_2026/planning/track_pipeline/config/track_extraction.yaml')
    parser.add_argument('--overwrite',action='store_true')
    args=parser.parse_args();config=yaml.safe_load(Path(args.config).read_text())
    root=find_workspace(__file__);source=root/config['source_run'];out=root/config['output']
    if out.exists() and any(out.iterdir()) and not args.overwrite:parser.error('Output exists; use a new output path or --overwrite')
    out.mkdir(parents=True,exist_ok=True)
    trajectory=np.loadtxt(source/'trajectory.tum');points=read_pcd(source/'map.pcd')
    evidence=project(points,trajectory,config)
    print('Floor',evidence['plane'],evidence['fit'],flush=True)
    origin=evidence['origin'];resolution=evidence['resolution']
    loop=extract_loop(evidence['usable'],resolution,origin,trajectory[:,1:3])
    s,centre,yaw,curvature,length=resample_closed(loop,config['sample_spacing_m'],config['smoothing_m'])
    left,right,ls,rs=ray_boundaries(centre,yaw,evidence['obstacles'],evidence['usable'],resolution,origin,config['max_boundary_ray_m'])
    left,right=track_limits(centre,yaw,curvature,evidence['usable'],resolution,origin,max_half_width=config['max_track_half_width_m'])
    left_width=np.linalg.norm(left-centre,axis=1);right_width=np.linalg.norm(right-centre,axis=1)
    metrics={'length_m':length,'samples':len(s),'spacing_m':length/len(s),'min_width_m':float(np.min(left_width+right_width)),
      'min_half_width_m':float(min(left_width.min(),right_width.min())),
      'max_abs_curvature_1_m':float(np.abs(curvature).max()),
      'left_wall_supported_fraction':float(ls.mean()),'right_wall_supported_fraction':float(rs.mean()),
      'centre_on_structural_obstacle':int(sample_grid(evidence['obstacles'],centre,resolution,origin).sum()),
      'centre_outside_floor_inferred_area':int((~sample_grid(evidence['usable'],centre,resolution,origin)).sum()),
      'centre_self_crossings':self_intersections(centre),'left_self_crossings':self_intersections(left),'right_self_crossings':self_intersections(right),
      'floor_plane':evidence['plane'].tolist(),'floor_fit':evidence['fit'],
      'source_run':config['source_run'],'frame':'camera_init','direction':'counterclockwise',
      'scope':'structural corridor for simulation; low obstacles, door openings and vehicle clearance require review',
      'vehicle_footprint_applied':False,'boundary_semantics':'conservative inscribed MPCC limits, not physical walls','geometric_margin_m':0.10,'max_abs_curvature_times_half_width':float(np.max(np.abs(curvature)*np.maximum(left_width,right_width)))}
    metrics.update(validate_strip(centre,left,right,evidence['usable'],resolution,origin))
    print(json.dumps(metrics,indent=2),flush=True)
    from .export import export
    export(out,source,config,evidence,trajectory,s,centre,yaw,curvature,left,right,ls,rs,metrics)
    if not metrics['geometry_valid']:
        raise RuntimeError('Track geometry validation failed; inspect overlay before use')

if __name__=='__main__':main()
