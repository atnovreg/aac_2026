"""Partition a binary PCD without changing retained point records."""
from pathlib import Path
import numpy as np
from planning.track_pipeline.projection import read_pcd


def split_pcd(source, kept_path, removed_path, removed):
    points=read_pcd(source)
    removed=np.asarray(removed,dtype=bool)
    if removed.shape!=(len(points),): raise ValueError('PCD selection size mismatch')
    header=[]
    with Path(source).open('rb') as file:
        while True:
            line=file.readline().decode('ascii')
            header.append(line)
            if line.startswith('DATA '): break
    for path,selection in [(kept_path,~removed),(removed_path,removed)]:
        count=int(selection.sum())
        with Path(path).open('xb') as out:
            for line in header:
                key=line.split()[0] if line.split() else ''
                if key in ('WIDTH','POINTS'): line=f'{key} {count}\n'
                if key=='HEIGHT': line='HEIGHT 1\n'
                out.write(line.encode('ascii'))
            for start in range(0,len(points),300000):
                out.write(points[start:start+300000][selection[start:start+300000]].tobytes())
