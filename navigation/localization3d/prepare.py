"""Prepare original measured 3D geometry and unchanged raceline inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
from planning.track_pipeline.projection import read_pcd
from voxel_sensor import VoxelWorld

HERE=Path(__file__).resolve().parent
from system.f1tenth.workspace import find_workspace
WS=find_workspace(__file__)
SOURCE=WS/'results/건지광장/dynamic_lio/20260913T100858860234Z_indoor_front180_20260913_083146/map_static.pcd'
INPUTS=HERE/'inputs'


def main(out):
    out.mkdir(exist_ok=False)
    a=read_pcd(SOURCE)
    xyz=np.c_[a['x'],a['y'],a['z']]
    with np.load(WS/'results/건지광장/map_edit/20261001_similarity_obstacles/drivable_area/area.npz') as d:
        plane=d['plane'].copy()
    height=xyz[:,2]-xyz[:,:2]@plane[:2]-plane[2]
    xyz=xyz[(height>=-.2)&(height<=6.)]
    world=VoxelWorld.from_points(xyz,voxel=.15)
    world.save(out/'map_geometry.npz')
    for name in ['track.csv','validation.json','soft_profile.npz','zones_map_xy.geojson','area_layers.npz']:
        shutil.copy2(INPUTS/name,out/name)
    manifest=dict(source=str(SOURCE),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        original_points=len(a),retained_points=len(xyz),occupied_voxels=int(world.occupied.sum()),
        grid_shape=world.shape.tolist(),voxel_m=world.voxel,floor_plane=plane.tolist(),
        height_filter_above_plane_m=[-.2,6.],
        geometry='3D occupied voxels from original measured static PCD; no planning boundaries/manual masks',
        sensor='Generic instantaneous regular 3D first-return grid; not validated MID360 pattern or surface reconstruction',
        input_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()})
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='New directory; existing data is never overwritten')
    main(parser.parse_args().output)
