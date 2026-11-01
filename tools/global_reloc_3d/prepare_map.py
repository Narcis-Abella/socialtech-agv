"""Prepare a ceiling-inclusive prior map from world.ply and map YAML only."""
import argparse
import json
from pathlib import Path

import numpy as np

from offline_s2 import map_transform, read_ply, transform, voxel, write_xyz


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--world',required=True)
    p.add_argument('--yaml',required=True)
    p.add_argument('--out',required=True)
    p.add_argument('--voxel',type=float,default=.20)
    a=p.parse_args()
    out=Path(a.out)
    out.mkdir(parents=True,exist_ok=False)
    raw=read_ply(a.world)
    prior=transform(raw,map_transform(a.yaml))
    keep=np.isfinite(prior).all(1) & (prior[:,2]>=-.5) & (prior[:,2]<=5.)
    cropped_points=int(np.sum(keep))
    prior=prior[keep]
    prior=voxel(prior,a.voxel)
    write_xyz(out/'map.xyz',prior)
    manifest={'world':str(a.world),'yaml':str(a.yaml),'raw_points':len(raw),
              'finite_cropped_points':cropped_points,
              'voxel_points':len(prior),'voxel_m':a.voxel,'frame':'map','truth_used':False}
    (out/'map_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':
    main()
