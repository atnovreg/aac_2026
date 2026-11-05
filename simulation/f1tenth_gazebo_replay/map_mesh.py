"""Display sampled PCD returns as small tetrahedra, without reconstructing surfaces."""
import struct
import numpy as np
from tools.track_pipeline.projection import read_pcd


def floor_z(xy, plane):
    return np.asarray(xy) @ np.asarray(plane[:2]) + plane[2]


def select_points(points, plane, voxel=.10, max_height=2.0, limit=120000):
    points = np.asarray(points)
    valid = np.isfinite(points).all(axis=1)
    points = points[valid]
    height = points[:, 2] - floor_z(points[:, :2], plane)
    points = points[(height >= -.12) & (height <= max_height)]
    if not len(points): raise ValueError('No map points within display height range')
    _, indices = np.unique(np.floor(points / voxel).astype(np.int64), axis=0, return_index=True)
    points = points[np.sort(indices)]
    if len(points) > limit:
        points = points[np.linspace(0, len(points)-1, limit, dtype=int)]
    return points


def write_stl(path, triangles):
    triangles = np.asarray(triangles, dtype=np.float32)
    normals = np.cross(triangles[:,1]-triangles[:,0], triangles[:,2]-triangles[:,0])
    length = np.linalg.norm(normals, axis=1)
    normals /= np.maximum(length[:,None], 1e-15)
    records = np.zeros(len(triangles), dtype=[('normal','<f4',3), ('vertices','<f4',(3,3)), ('attr','<u2')])
    records['normal'] = normals; records['vertices'] = triangles
    with path.open('wb') as file:
        file.write(b'PCD display geometry'.ljust(80, b'\0'))
        file.write(struct.pack('<I', len(records))); file.write(records.tobytes())


def map_meshes(source, output, plane, max_height=2.0):
    raw = read_pcd(source)
    stride = max(1, len(raw)//2000000)
    sampled = raw[::stride]
    points = select_points(np.c_[sampled['x'],sampled['y'],sampled['z']], plane, max_height=max_height)
    height = points[:,2]-floor_z(points[:,:2], plane)
    colors = ['.42 .48 .55 1', '.18 .53 .62 1', '.25 .66 .55 1', '.68 .73 .30 1']
    vertices = np.array([[1,1,1],[-1,-1,1],[-1,1,-1],[1,-1,-1]])*.035
    faces = np.array([[0,2,1],[0,1,3],[0,3,2],[1,2,3]])
    meshes = []
    bins = np.digitize(height, [.15,.8,1.4])
    for index, color in enumerate(colors):
        group = points[bins == index]
        if not len(group): continue
        triangles = (group[:,None,None,:] + vertices[faces][None,:,:,:]).reshape(-1,3,3)
        name = f'pointcloud_{index}.stl'
        write_stl(output/name, triangles)
        meshes.append((name, color))
    return meshes, {'source_pcd':str(source), 'source_points':len(raw), 'display_points':len(points),
                    'sample_stride':stride, 'voxel_m':.10, 'height_above_floor_m':[-.12,max_height],
                    'representation':'sampled original XYZ returns displayed as 7 cm tetrahedra'}
