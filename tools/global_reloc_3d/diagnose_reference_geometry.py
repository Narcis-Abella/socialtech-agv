"""Evaluator-only oracle overlap diagnostic; NEVER mount this in a method run."""
import argparse
import json
from pathlib import Path

import numpy as np

from evaluate import reference_at
from offline_s2 import map_transform
from verify import score


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--run',required=True)
    p.add_argument('--reference',required=True)
    p.add_argument('--map-yaml',required=True)
    p.add_argument('--out',required=True)
    a=p.parse_args()
    run=Path(a.run)
    manifest=json.loads((run/'manifest.json').read_text())
    t=manifest['stamps_s'][0]
    reference=np.loadtxt(a.reference,usecols=range(8))
    T=map_transform(a.map_yaml)@reference_at(reference,t)
    if manifest['frame_fit']=='livox_lidar_stationary':
        E=np.eye(4);E[:3,3]=[-.011,-.02329,.04412]
        T=T@E
    elif manifest['frame_fit']!='local_first_imu':
        raise ValueError('unsupported local frame')
    check=np.loadtxt(run/'check.xyz')
    prior=np.loadtxt(run/'map.xyz')
    result={'oracle_geometry':score(prior,check,T),
            'reference_time_s':t,'purpose':'evaluator_only_map_overlap_diagnosis',
            'method_used_this_transform':False}
    Path(a.out).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
