"""Evaluator-only negative check for a frozen IMU stationarity rule.

Uses disjoint 2 s bins anchored at the first reference timestamp. The reference
labels bins only after the rule is fixed from separate static recordings. No
candidate or executor receives these labels, statistics or time selections.
"""
import argparse
import gzip
import json
import math
from pathlib import Path

import numpy as np

from window_inventory import CONFIG, NS, canonical, crosses, imu_pass, imu_stats, load_reference


def validate(root):
    reports=[]
    for dataset in ('eco01_1x','hospital_02_r05b','eco03'):
        base=root/dataset
        data=json.loads(gzip.decompress((base/'sensor-index.json.gz').read_bytes()))
        t=np.asarray([r[1] for r in data['imu']],dtype=np.int64)
        values=np.asarray([r[2:] for r in data['imu']],dtype=float)
        ref,_=load_reference(base/'alidarState.txt',base/'map.yaml')
        median_ns=float(np.median(np.diff(t)))
        hard=[[int(t[i]),int(t[i+1])] for i in np.flatnonzero(np.diff(t)>CONFIG['imu_hard_gap_factor']*median_ns)]
        categories={k:[] for k in ('stationary_B','clearly_moving','ambiguous')}
        excluded=0
        for start in range(int(ref.t[0]),int(ref.t[-1])-2*NS+1,2*NS):
            end=start+2*NS
            if crosses(hard,start,end): excluded+=1; continue
            try: p,q,_=ref.segment(start,end-1)
            except ValueError: excluded+=1; continue
            i,j=np.searchsorted(t,[start,end])
            if j-i<math.ceil(.95*math.ceil(2*NS/median_ns)):
                excluded+=1; continue
            dis=float(np.linalg.norm(p-p[0],axis=1).max())
            angle=float(np.degrees(2*np.arccos(np.clip(np.abs(q@q[0]),0,1))).max())
            label=('stationary_B' if dis<=.05 and angle<=2 else
                   'clearly_moving' if dis>=.4 or angle>=10 else 'ambiguous')
            stats=imu_stats(values[i:j],CONFIG['accel_scale'])
            categories[label].append((imu_pass(stats),stats['gyro_centered_residual_norm_p95_rad_s'],
                                      stats['gyro_norm_p95_rad_s']))
        def summary(rows):
            return {'n':len(rows),'passed':sum(r[0] for r in rows),
                    'centered_p95_median':float(np.median([r[1] for r in rows])) if rows else None,
                    'raw_p95_median':float(np.median([r[2] for r in rows])) if rows else None}
        reports.append({'dataset_id':dataset,'bins_2s':{k:summary(v) for k,v in categories.items()},
                        'excluded_bad_imu_or_reference':excluded})
    return reports


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-root',type=Path,required=True)
    args=parser.parse_args()
    print(canonical(validate(args.input_root)).decode())
