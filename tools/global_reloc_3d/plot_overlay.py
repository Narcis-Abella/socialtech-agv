"""Plot a frozen estimate over its prior cloud; no scoring or pose selection."""
import argparse
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from offline_s2 import transform


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--map',required=True)
    p.add_argument('--fit',required=True)
    p.add_argument('--result',required=True)
    p.add_argument('--out',required=True)
    a=p.parse_args()
    world=np.loadtxt(a.map)
    fit=np.loadtxt(a.fit)
    T=np.array(json.load(open(a.result))['T_map_lidar_refined'])
    aligned=transform(fit,T)
    x,y=T[0,3],T[1,3]
    near=world[(abs(world[:,0]-x)<8)&(abs(world[:,1]-y)<8)]
    fig,axes=plt.subplots(1,2,figsize=(8,3.6),layout='constrained')
    axes[0].scatter(world[:,0],world[:,1],s=.14,c='0.65',rasterized=True)
    axes[0].scatter(aligned[:,0],aligned[:,1],s=.9,c='#d94841',alpha=.55,rasterized=True)
    axes[0].plot([x],[y],'ko',ms=3)
    axes[0].set_title('Map XY (full)')
    axes[0].set_aspect('equal')
    axes[1].scatter(near[:,0],near[:,1],s=.35,c='0.65',rasterized=True)
    axes[1].scatter(aligned[:,0],aligned[:,1],s=1,c='#d94841',alpha=.55,rasterized=True)
    axes[1].plot([x],[y],'ko',ms=3)
    axes[1].set_xlim(x-8,x+8); axes[1].set_ylim(y-8,y+8)
    axes[1].set_title('Selected pose, 16 m window')
    for ax in axes: ax.set_xlabel('map x (m)'); ax.set_ylabel('map y (m)')
    fig.savefig(a.out,dpi=140)


if __name__=='__main__':
    main()
