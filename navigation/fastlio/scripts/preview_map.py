#!/usr/bin/env python3
"""Validate a binary PCD and render a bounded-size top/side map preview."""
import argparse
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser()
parser.add_argument('run',type=Path)
args=parser.parse_args()
with (args.run/'map.pcd').open('rb') as file:
    header={}
    while True:
        line=file.readline().decode('ascii').strip()
        if not line: raise ValueError('Incomplete PCD')
        if line.startswith('#'): continue
        key,*values=line.split(); header[key]=values
        if key=='DATA': break
    if header['DATA']!=['binary']: raise ValueError('Expected binary PCD')
    mapping={('F','4'):'<f4',('F','8'):'<f8',('U','4'):'<u4',('I','4'):'<i4'}
    dtype=np.dtype([(name,mapping[(kind,size)],int(count)) if int(count)>1 else (name,mapping[(kind,size)]) for name,kind,size,count in zip(header['FIELDS'],header['TYPE'],header['SIZE'],header['COUNT'])])
    data=np.fromfile(file,dtype=dtype)
if len(data)!=int(header['POINTS'][0]): raise ValueError('PCD point count mismatch')
for axis in ['x','y','z']:
    if not np.isfinite(data[axis]).all(): raise ValueError('Non-finite map coordinates')
bounds={axis:[float(data[axis].min()),float(data[axis].max())] for axis in ['x','y','z']}
(args.run/'map-validation.json').write_text(json.dumps({'points':len(data),'finite':True,'bounds_m':bounds},indent=2)+'\n')
# Deterministic sampling bounds plotting memory without modifying the PCD.
cloud=data[::max(1,len(data)//250000)]
trajectory=np.loadtxt(args.run/'trajectory.tum',ndmin=2)
fig,axes=plt.subplots(1,2,figsize=(15,7),constrained_layout=True)
lo,hi=np.percentile(cloud['z'],[2,98])
for ax,yaxis,pose_col,title in [(axes[0],'y',2,'Top view'),(axes[1],'z',3,'Side view')]:
    points=ax.scatter(cloud['x'],cloud[yaxis],c=cloud['z'],s=0.2,cmap='viridis',vmin=lo,vmax=hi,rasterized=True)
    ax.plot(trajectory[:,1],trajectory[:,pose_col],color='red',lw=1,label='IMU trajectory')
    ax.scatter(trajectory[0,1],trajectory[0,pose_col],s=35,c='orange',marker='*',label='Start')
    ax.set(xlabel='x (m)',ylabel=f'{yaxis} (m)',title=title,aspect='equal'); ax.legend(); ax.grid(alpha=.2)
fig.colorbar(points,ax=axes,label='Height z (m)',shrink=.6)
fig.suptitle(args.run.parent.name+' — FAST-LIO2 (assumed MID-360 calibration)')
fig.savefig(args.run/'preview.png',dpi=160)
print(json.dumps({'run':str(args.run),'points':len(data),'bounds_m':bounds}))
