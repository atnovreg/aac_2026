"""Conservative, offline voxel visibility evidence (not human segmentation).

Timestamp registration belongs to the caller. Actual measured ray segments can
provide free-space evidence; missing returns and occluded voxels provide none.
The small compiled DDA kernel is built in a private temporary directory with the
system C++ compiler, requires no ROS dependencies, and never modifies inputs.
"""
from __future__ import annotations

import ctypes
import functools
from pathlib import Path
import subprocess
import tempfile
import numpy as np


_CPP = r'''
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

extern "C" void observe_rays(
 const double *origins, const double *ends, int64_t n, int shared,
 const int64_t *shape, const int64_t *qmin, double voxel, double margin,
 double hit_radius, double free_radius, double min_range, const int32_t *grid,
 uint32_t *hit_epoch, uint32_t *free_epoch, uint32_t *hits, uint32_t *frees,
 int64_t candidates, uint32_t epoch) {
 const auto index = [&](int64_t x, int64_t y, int64_t z) -> int64_t {
   if (x<0 || y<0 || z<0 || x>=shape[0] || y>=shape[1] || z>=shape[2]) return -1;
   return (x*shape[1]+y)*shape[2]+z;
 };
 const int radius = int(std::ceil(hit_radius/voxel));
 // Support is collected first, so input ray order cannot erase a supported cell.
 for (int64_t j=0; j<n; ++j) {
   const double *o=origins+(shared?0:3*j), *p=ends+3*j;
   double distance2=0; bool finite=true;
   for(int k=0;k<3;++k) {finite &= std::isfinite(o[k])&&std::isfinite(p[k]); distance2+=(p[k]-o[k])*(p[k]-o[k]);}
   if(!finite || distance2<min_range*min_range) continue;
   int64_t q[3]; for(int k=0;k<3;++k) q[k]=int64_t(std::floor(p[k]/voxel))-qmin[k];
   for(int dx=-radius; dx<=radius; ++dx) for(int dy=-radius; dy<=radius; ++dy) for(int dz=-radius; dz<=radius; ++dz) {
     int64_t x=q[0]+dx,y=q[1]+dy,z=q[2]+dz, flat=index(x,y,z);
     if(flat<0 || grid[flat]<0) continue;
     double d2=0; const int64_t xyz[3]={x,y,z};
     for(int k=0;k<3;++k) {double v=(xyz[k]+qmin[k]+.5)*voxel-p[k]; d2+=v*v;}
     if(d2<=hit_radius*hit_radius) hit_epoch[grid[flat]]=epoch;
   }
 }
 for(int64_t j=0;j<n;++j) {
   const double *o=origins+(shared?0:3*j), *p=ends+3*j;
   double d[3], length2=0; bool finite=true;
   for(int k=0;k<3;++k) {finite &= std::isfinite(o[k])&&std::isfinite(p[k]); d[k]=p[k]-o[k];length2+=d[k]*d[k];}
   const double length=std::sqrt(length2);
   if(!finite || length<=min_range+margin) continue;
   for(int k=0;k<3;++k) d[k]/=length;
   double enter=min_range, leave=length-margin;
   bool intersects=true;
   for(int k=0;k<3;++k) {
     double low=qmin[k]*voxel, high=(qmin[k]+shape[k])*voxel;
     if(std::abs(d[k])<1e-15) { if(o[k]<low || o[k]>=high) intersects=false; }
     else {double a=(low-o[k])/d[k], b=(high-o[k])/d[k]; if(a>b)std::swap(a,b); enter=std::max(enter,a); leave=std::min(leave,b);}
   }
   if(!intersects || leave<=enter+1e-8) continue;
   int64_t cell[3]; int step[3]; double next[3], delta[3];
   for(int k=0;k<3;++k) {
     cell[k]=int64_t(std::floor((o[k]+(enter+1e-8)*d[k])/voxel))-qmin[k];
     if(std::abs(d[k])<1e-15) {step[k]=0;next[k]=delta[k]=std::numeric_limits<double>::infinity();}
     else {step[k]=d[k]>0?1:-1; double edge=(cell[k]+qmin[k]+(step[k]>0?1:0))*voxel;next[k]=(edge-o[k])/d[k];delta[k]=voxel/std::abs(d[k]);}
   }
   while(enter<leave) {
     const int64_t flat=index(cell[0],cell[1],cell[2]);
     if(flat<0) break;
     const double exit=std::min({next[0],next[1],next[2],leave});
     const int32_t c=grid[flat];
     if(c>=0 && hit_epoch[c]!=epoch && free_epoch[c]!=epoch) {
       double center[3],along=0;
       for(int k=0;k<3;++k) {center[k]=(cell[k]+qmin[k]+.5)*voxel;along+=(center[k]-o[k])*d[k];}
       along=std::max(enter,std::min(exit,along));
       double distance2=0;for(int k=0;k<3;++k) {const double e=center[k]-o[k]-along*d[k];distance2+=e*e;}
       if(distance2<=free_radius*free_radius) free_epoch[c]=epoch;
     }
     if(exit>=leave) break;
     for(int k=0;k<3;++k) if(next[k]<=exit+1e-10) {cell[k]+=step[k];next[k]+=delta[k];}
     enter=exit;
   }
 }
 for(int64_t c=0;c<candidates;++c) {
   if(hit_epoch[c]==epoch) ++hits[c];
   else if(free_epoch[c]==epoch) ++frees[c];
 }
}
'''


@functools.lru_cache(maxsize=1)
def _kernel():
    directory = tempfile.TemporaryDirectory(prefix='f1tenth-visibility-')
    path = Path(directory.name)
    (path / 'visibility.cpp').write_text(_CPP)
    subprocess.run(['g++', '-std=c++17', '-O3', '-shared', '-fPIC',
                    str(path / 'visibility.cpp'), '-o', str(path / 'visibility.so')],
                   check=True, capture_output=True, text=True)
    library = ctypes.CDLL(str(path / 'visibility.so'))
    f64 = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
    i64 = np.ctypeslib.ndpointer(dtype=np.int64, flags='C_CONTIGUOUS')
    i32 = np.ctypeslib.ndpointer(dtype=np.int32, flags='C_CONTIGUOUS')
    u32 = np.ctypeslib.ndpointer(dtype=np.uint32, flags='C_CONTIGUOUS')
    library.observe_rays.argtypes = [f64, f64, ctypes.c_int64, ctypes.c_int,
                                    i64, i64, *([ctypes.c_double] * 5), i32,
                                    u32, u32, u32, u32, ctypes.c_int64, ctypes.c_uint32]
    library.observe_rays.restype = None
    return directory, library


class TemporalVoxelEvidence:
    """Classify candidate voxels only after repeated measured free-space evidence.

    ``voxel_keys`` are integer global ``floor(xyz / voxel_size)`` coordinates.
    Public evidence arrays share their order; ``finalize()`` returns this order.
    A hit in any ray protects that voxel for the complete observation frame.
    ``min_static_frames`` protects repeatedly observed structure even when its
    free-space ratio is high; set to None explicitly to disable this protection.
    """
    def __init__(self, candidate_xyz, voxel_size=.10, floor_plane=(0., 0., 0.),
                 height_range=(.15, 1.9), endpoint_margin=.25,
                 min_free_frames=3, min_free_ratio=.8, min_frame_gap=.5,
                 hit_radius=.15, free_radius=None, min_sensor_range=.5,
                 min_static_frames=10, max_grid_cells=100_000_000):
        self.voxel_size = float(voxel_size)
        self.floor_plane = np.asarray(floor_plane, dtype=float)
        self.height_range = tuple(float(x) for x in height_range)
        self.endpoint_margin = float(endpoint_margin)
        self.min_free_frames = int(min_free_frames)
        self.min_free_ratio = float(min_free_ratio)
        self.min_frame_gap = float(min_frame_gap)
        self.hit_radius = float(hit_radius)
        self.free_radius = .35 * self.voxel_size if free_radius is None else float(free_radius)
        self.min_sensor_range = float(min_sensor_range)
        self.min_static_frames = min_static_frames
        values = [self.voxel_size, self.endpoint_margin, self.min_free_ratio,
                  self.min_frame_gap, self.hit_radius, self.free_radius, self.min_sensor_range]
        if (not np.isfinite(values).all() or self.voxel_size <= 0 or
                min(values[1:]) < 0 or not 0 <= self.min_free_ratio <= 1 or
                self.min_free_frames < 1 or self.floor_plane.shape != (3,) or
                not np.isfinite(self.floor_plane).all() or
                len(self.height_range) != 2 or not np.isfinite(self.height_range).all() or
                self.height_range[0] >= self.height_range[1] or
                (min_static_frames is not None and min_static_frames < 1)):
            raise ValueError('Invalid temporal visibility parameters')
        points = self._points(candidate_xyz)
        points = points[self._eligible(points)]
        self.voxel_keys = np.unique(np.floor(points / self.voxel_size).astype(np.int64), axis=0)
        self.voxel_centers = (self.voxel_keys + .5) * self.voxel_size
        n = len(self.voxel_keys)
        self.free_counts = np.zeros(n, dtype=np.uint32)
        self.hit_counts = np.zeros(n, dtype=np.uint32)
        self._free_epoch = np.zeros(n, dtype=np.uint32)
        self._hit_epoch = np.zeros(n, dtype=np.uint32)
        self.frames = 0
        self.skipped_frames = 0
        self.rays = 0
        self._last_time = None
        if n:
            self._qmin = np.ascontiguousarray(self.voxel_keys.min(axis=0))
            self._shape = np.ascontiguousarray(self.voxel_keys.max(axis=0) - self._qmin + 1)
            cells = int(np.prod(self._shape, dtype=object))
            if cells > max_grid_cells:
                raise ValueError(f'Candidate grid needs {cells} cells; crop invalid/outlier map coordinates first')
            self._grid = np.full(cells, -1, dtype=np.int32)
            self._grid[self._flat(self.voxel_keys)] = np.arange(n, dtype=np.int32)
        else:
            self._qmin = np.zeros(3, dtype=np.int64)
            self._shape = np.ones(3, dtype=np.int64)
            self._grid = np.full(1, -1, dtype=np.int32)

    @staticmethod
    def _points(points):
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('Expected xyz array of shape (N,3)')
        return points

    def _eligible(self, points):
        height = points[:, 2] - (points[:, :2] @ self.floor_plane[:2] + self.floor_plane[2])
        return (np.isfinite(points).all(axis=1) & (height >= self.height_range[0]) &
                (height <= self.height_range[1]))

    def _flat(self, keys):
        q = keys - self._qmin
        return (q[:, 0] * self._shape[1] + q[:, 1]) * self._shape[2] + q[:, 2]

    def observe(self, origins_xyz, endpoints_xyz, timestamp):
        """Add one frame. Return False when less than min_frame_gap has elapsed."""
        timestamp = float(timestamp)
        if not np.isfinite(timestamp):
            raise ValueError('Frame time must be finite')
        if self._last_time is not None and timestamp < self._last_time:
            raise ValueError('Observation frames must be chronologically ordered')
        if self._last_time is not None and timestamp - self._last_time < self.min_frame_gap - 1e-9:
            self.skipped_frames += 1
            return False
        endpoints = np.ascontiguousarray(self._points(endpoints_xyz))
        origins = np.ascontiguousarray(origins_xyz, dtype=np.float64)
        if origins.shape not in ((3,), (len(endpoints), 3)):
            raise ValueError('Origins must have shape (3,) or match endpoints (N,3)')
        if self.frames >= np.iinfo(np.uint32).max - 1:
            raise OverflowError('Too many observation frames')
        self.frames += 1
        self.rays += len(endpoints)
        self._last_time = timestamp
        if len(self.voxel_keys) and len(endpoints):
            _, library = _kernel()
            library.observe_rays(origins, endpoints, len(endpoints), int(origins.ndim == 1),
                                 self._shape, self._qmin, self.voxel_size, self.endpoint_margin,
                                 self.hit_radius, self.free_radius, self.min_sensor_range,
                                 self._grid, self._hit_epoch, self._free_epoch,
                                 self.hit_counts, self.free_counts, len(self.voxel_keys), self.frames)
        return True

    def finalize(self):
        """Return a fresh classification mask, without changing evidence."""
        denominator = self.free_counts.astype(float) + self.hit_counts
        ratio = np.divide(self.free_counts, denominator, out=np.zeros_like(denominator), where=denominator > 0)
        dynamic = (self.free_counts >= self.min_free_frames) & (ratio >= self.min_free_ratio)
        if self.min_static_frames is not None:
            dynamic &= self.hit_counts < self.min_static_frames
        return dynamic

    def removal_mask(self, world_xyz):
        points = self._points(world_xyz)
        removed = np.zeros(len(points), dtype=bool)
        valid = self._eligible(points)
        ids = np.flatnonzero(valid)
        keys = np.floor(points[valid] / self.voxel_size).astype(np.int64)
        q = keys - self._qmin
        inside = ((q >= 0) & (q < self._shape)).all(axis=1)
        ids = ids[inside]
        candidate_ids = self._grid[self._flat(keys[inside])]
        found = candidate_ids >= 0
        removed[ids[found]] = self.finalize()[candidate_ids[found]]
        return removed

    def summary(self):
        return {'frames': self.frames, 'skipped_frames': self.skipped_frames, 'rays': self.rays,
                'candidate_voxels': len(self.voxel_keys), 'removed_voxels': int(self.finalize().sum()),
                'voxel_size_m': self.voxel_size, 'endpoint_margin_m': self.endpoint_margin,
                'hit_radius_m': self.hit_radius, 'free_radius_m': self.free_radius,
                'min_sensor_range_m': self.min_sensor_range, 'height_range_m': self.height_range,
                'min_free_frames': self.min_free_frames, 'min_free_ratio': self.min_free_ratio,
                'min_static_frames': self.min_static_frames, 'min_frame_gap_s': self.min_frame_gap,
                'method': 'Repeated measured ray passages; not semantic human labels'}
