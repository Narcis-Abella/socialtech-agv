#!/usr/bin/env python3
"""POST-RUN evaluator-only scan/map diagnostic on DEVELOPMENT data.

This script deliberately reads reference poses. Never mount or invoke it in a
candidate/executor container, or use its output to construct an online scan.
It only diagnoses geometry after a trial and cannot provide a final benchmark.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from amcl_acceptance import Occupancy, Scan, map_support
from pose_eval import read_map_yaml, read_pgm, reference_map
from scan_projection_sweep import configurations, read_window
from sensor_relative_scan import project_scan


def reference_at(t_ns,t_sec,poses,max_bracket_s=.20):
    t=t_ns/1e9
    idx=int(np.searchsorted(t_sec,t))
    if idx<len(t_sec) and abs(t_sec[idx]-t)<1e-9:
        return tuple(float(v) for v in poses[idx])
    if idx==0 or idx==len(t_sec):return None
    lo,hi=idx-1,idx
    span=float(t_sec[hi]-t_sec[lo])
    if span<=0 or span>max_bracket_s:return None
    fraction=(t-t_sec[lo])/span
    x=(1-fraction)*poses[lo,0]+fraction*poses[hi,0]
    y=(1-fraction)*poses[lo,1]+fraction*poses[hi,1]
    dyaw=math.atan2(math.sin(poses[hi,2]-poses[lo,2]),
                    math.cos(poses[hi,2]-poses[lo,2]))
    yaw=math.atan2(math.sin(poses[lo,2]+fraction*dyaw),
                   math.cos(poses[lo,2]+fraction*dyaw))
    return float(x),float(y),yaw


def occupancy_from_pgm(pgm,yaml):
    raw=read_pgm(pgm)
    # Nav2 trinary map: black occupied, near-white free, intermediate unknown.
    if 'mode: trinary' not in Path(yaml).read_text() or 'negate: 0' not in Path(yaml).read_text():
        raise ValueError('expected trinary, non-negated PGM')
    meta=read_map_yaml(yaml)
    occupied=raw<round(255*(1-.65))
    data=np.where(occupied,100,0).astype(np.int16)
    return Occupancy(np.flipud(data),meta['resolution'],*meta['origin'],0.,65)


def run(args):
    clouds,gravity,metadata=read_window(args.bag,args.start_ns,args.end_ns,
                                        args.imu_median_ns,args.frames)
    map_meta=read_map_yaml(args.map_yaml)
    t_ref,ref=reference_map(args.reference,map_meta['T_map_world'])
    occupied=occupancy_from_pgm(args.map_pgm,args.map_yaml)
    selected={c['config_id']:c for c in configurations() if c['config_id'] in args.config_ids}
    if set(selected)!=set(args.config_ids):raise ValueError('unknown config ID')
    rows=[]
    for config_id in args.config_ids:
        config=selected[config_id]
        for stamp,cloud in zip(metadata['selected_sensor_stamps_ns'],clouds):
            truth=reference_at(stamp,t_ref,ref)
            if truth is None:
                rows.append({'config_id':config_id,'sensor_stamp_ns':stamp,
                             'status':'reference_gap_or_out_of_bounds','map_support_truth':None})
                continue
            projection=project_scan(cloud,gravity,config['height_m'],
                min_height=config['band_min_m'],max_height=config['band_max_m'],
                sensor_relative_band_m=config['sensor_relative_band_m'],
                fov_deg=config['fov_deg'],fov_center_deg=config['fov_center_deg'],
                angle_increment=math.radians(config['angle_increment_deg']),
                range_min=config['range_min'],range_max=config['range_max'])
            scan=Scan(projection['ranges'],projection['angle_min'],
                      projection['angle_increment'],projection['range_min'],
                      projection['range_max'])
            rows.append({'config_id':config_id,'sensor_stamp_ns':stamp,
                         'status':'scored','map_support_truth':map_support(scan,truth,occupied),
                         'finite_beams':projection['finite_count']})
    return rows,{'schema_version':1,'classification':'POST-RUN EVALUATOR ONLY; TUNED ON DEVELOPMENT',
                 'bag_basename':args.bag.name,'map_basename':args.map_yaml.name,
                 'reference_basename':args.reference.name,'selected_sensor_stamps_ns':metadata['selected_sensor_stamps_ns'],
                 'config_ids':args.config_ids,'max_reference_bracket_s':.20}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag',type=Path);ap.add_argument('map_pgm',type=Path)
    ap.add_argument('map_yaml',type=Path);ap.add_argument('reference',type=Path)
    ap.add_argument('start_ns',type=int);ap.add_argument('end_ns',type=int)
    ap.add_argument('imu_median_ns',type=float);ap.add_argument('output_prefix',type=Path)
    ap.add_argument('--frames',type=int,default=7)
    ap.add_argument('--config-ids',nargs='+',default=['h05_b10_180','h10_b10_180',
        'h15_b10_180','h20_b10_180','h10_b05_075','h10_b30_125',
        'f090_c000','f180_c000','f270_c000','range075_20'])
    args=ap.parse_args()
    rows,meta=run(args)
    csv_path=Path(str(args.output_prefix)+'.csv')
    json_path=Path(str(args.output_prefix)+'.json')
    if csv_path.exists() or json_path.exists():raise FileExistsError(args.output_prefix)
    fields=['config_id','sensor_stamp_ns','status','map_support_truth','finite_beams']
    with csv_path.open('x',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    with json_path.open('x',encoding='utf-8') as stream:
        json.dump(meta,stream,sort_keys=True,indent=2);stream.write('\n')
    print(json.dumps({'rows':len(rows),'scored':sum(r['status']=='scored' for r in rows),
                      'csv':str(csv_path)},sort_keys=True))


if __name__=='__main__':main()
