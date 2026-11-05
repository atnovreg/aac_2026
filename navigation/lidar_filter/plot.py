"""Scientific before/after plots with identical original sampling and axes."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from tools.track_pipeline.projection import read_pcd,xyz
from .__main__ import SavedMask


def main():
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);args=parser.parse_args()
    out=args.output;mask=SavedMask(out)
    cloud=xyz(read_pcd(Path(mask.manifest['source_run'])/'map.pcd')[::12])
    removed=mask.mask(cloud);h=cloud[:,2]-cloud[:,:2]@mask.plane[:2]-mask.plane[2]
    visible=(h>.15)&(h<1.9)
    fig,axs=plt.subplots(2,3,figsize=(15,9),layout='constrained')
    for col,(selection,title,color) in enumerate([(visible,'Original','#334155'),(visible&~removed,'After temporal filtering','#334155'),(visible&removed,'Removed returns','#dc2626')]):
        for row,lims in enumerate([(-5,18,-4,15),(10,17,7,14)]):
            ax=axs[row,col];ax.scatter(cloud[selection,0],cloud[selection,1],s=.6,c=color,rasterized=True)
            ax.set(xlim=lims[:2],ylim=lims[2:],aspect='equal',title=title,xlabel='x [m]',ylabel='y [m]');ax.grid(alpha=.15)
    fig.suptitle('Temporal filtering of measured 3D map — same point sampling, height 0.15–1.90 m',fontsize=14)
    fig.savefig(out/'before_after.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(11,5.5),layout='constrained')
    for ax,selection,title in zip(axs,[visible,visible&~removed],['Original corner','After temporal filtering']):
        ax.scatter(cloud[selection,0],cloud[selection,1],s=1,c='#334155',rasterized=True)
        ax.set(xlim=(10,17),ylim=(7,14),aspect='equal',title=title,xlabel='x [m]',ylabel='y [m]');ax.grid(alpha=.15)
    fig.savefig(out/'corner_before_after.png',dpi=180);plt.close(fig)

if __name__=='__main__':main()
