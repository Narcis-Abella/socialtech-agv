#!/usr/bin/env python3
"""Sensor-only exploratory projection sweep for a stationary Livox window.

This tool has no map or reference input. It produces descriptive scan metrics,
not a localization score. Its fixed grid is deliberately broad and small.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from sensor_relative_scan import conditioning_gravity, project_scan


def configurations():
    """Stable, diverse coarse grid; no result-dependent parameter selection."""
    base={'fov_deg':360., 'fov_center_deg':0., 'angle_increment_deg':.5,
          'range_min':.5, 'range_max':30.}
    out=[]
    bands=((.05,.75),(.10,1.80),(.30,1.25),(.40,1.80))
    for tenth in range(1,21):
        for lo,hi in bands:
            out.append({'config_id':f'h{tenth:02d}_b{int(lo*100):02d}_{int(hi*100):03d}',
                        'height_m':tenth/10.,'band_min_m':lo,'band_max_m':hi,**base})
    for fov in (90.,120.,180.,270.):
        for center in (0.,180.):
            out.append({'config_id':f'f{int(fov):03d}_c{int(center):03d}',
                        'height_m':1.,'band_min_m':.10,'band_max_m':1.80,
                        **{**base,'fov_deg':fov,'fov_center_deg':center}})
    for resolution in (.25,1.,2.):
        out.append({'config_id':f'r{int(resolution*100):03d}',
                    'height_m':1.,'band_min_m':.10,'band_max_m':1.80,
                    **{**base,'angle_increment_deg':resolution}})
    for rmin,rmax in ((.2,8.),(.3,12.),(.75,20.),(1.,30.)):
        out.append({'config_id':f'range{int(rmin*100):03d}_{int(rmax):02d}',
                    'height_m':1.,'band_min_m':.10,'band_max_m':1.80,
                    **{**base,'range_min':rmin,'range_max':rmax}})
    for lo,hi in ((-1.2,-.4),(-.8,.2),(-.3,.5),(.1,.9)):
        out.append({'config_id':f'sensorband_{len(out):03d}',
                    'height_m':1.,'band_min_m':lo,'band_max_m':hi,
                    'sensor_relative_band_m':(lo,hi),**base})
    for config in out:
        config.setdefault('sensor_relative_band_m',None)
    return out


def summarize_config(config,clouds,gravity):
    scans=[]
    for xyz in clouds:
        scans.append(project_scan(xyz,gravity,config['height_m'],
            min_height=config['band_min_m'],max_height=config['band_max_m'],
            sensor_relative_band_m=config.get('sensor_relative_band_m'),
            fov_deg=config['fov_deg'],fov_center_deg=config['fov_center_deg'],
            angle_increment=math.radians(config['angle_increment_deg']),
            range_min=config['range_min'],range_max=config['range_max']))
    ranges=np.stack([s['ranges'] for s in scans]);finite=np.isfinite(ranges)
    n=finite.sum(axis=1);covered=finite.sum(axis=0)>0
    stable=[]
    for a,b in zip(finite[:-1],finite[1:]):
        union=np.count_nonzero(a|b)
        stable.append(np.count_nonzero(a&b)/union if union else 0.)
    near=(finite & (ranges<1.5)).sum(axis=1)
    return {**config,'frames':len(scans),'beams':scans[0]['beam_count'],
            'finite_median':float(np.median(n)),
            'finite_p05':float(np.quantile(n,.05)),
            'finite_p95':float(np.quantile(n,.95)),
            'finite_fraction_median':float(np.median(n)/scans[0]['beam_count']),
            'covered_fraction':float(np.mean(covered)),
            'near_fraction_median':float(np.median(near/np.maximum(n,1))),
            'adjacent_finite_jaccard_median':float(np.median(stable)) if stable else None,
            'band_points_median':float(np.median([s['band_point_count'] for s in scans]))}


def read_window(bag,start_ns,end_ns,median_ns,max_frames):
    from rosbags.highlevel import AnyReader
    with AnyReader([Path(bag)]) as reader:
        cs={c.topic:c for c in reader.connections if c.topic in ('/livox/lidar','/livox/imu')}
        if set(cs)!={'/livox/lidar','/livox/imu'}:
            raise ValueError('required sensor topics absent')
        imu=[];lidar=[];frames=set()
        for c,_,raw in reader.messages(connections=list(cs.values()),
                start=max(0,start_ns-2_500_000_000),stop=end_ns+500_000_000):
            m=reader.deserialize(raw,c.msgtype)
            stamp=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
            if c.topic=='/livox/imu' and start_ns-2_000_000_000<=stamp<start_ns:
                a=m.linear_acceleration
                imu.append((stamp,(a.x,a.y,a.z)));frames.add(m.header.frame_id)
            elif c.topic=='/livox/lidar' and start_ns<=stamp<end_ns:
                lidar.append((stamp,np.asarray([(p.x,p.y,p.z) for p in m.points],float)))
                frames.add(m.header.frame_id)
        if len(frames)!=1:raise ValueError('LiDAR/IMU frame mismatch')
        gravity,meta=conditioning_gravity(imu,start_ns-2_000_000_000,start_ns,median_ns)
        if gravity is None:raise ValueError(f'IMU conditioning failed: {meta}')
        if not lidar:raise ValueError('no LiDAR in selected window')
        indices=np.unique(np.linspace(0,len(lidar)-1,min(max_frames,len(lidar)),dtype=int))
        return [lidar[int(i)][1] for i in indices],gravity,{
            'lidar_frames_total':len(lidar),'selected_sensor_stamps_ns':[int(lidar[int(i)][0]) for i in indices],
            'frame_id':next(iter(frames)),'imu_conditioning':meta}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag',type=Path);ap.add_argument('start_ns',type=int)
    ap.add_argument('end_ns',type=int);ap.add_argument('median_imu_ns',type=float)
    ap.add_argument('output_prefix',type=Path);ap.add_argument('--frames',type=int,default=7)
    args=ap.parse_args()
    if args.end_ns<=args.start_ns or args.frames<1:ap.error('invalid interval or frames')
    clouds,gravity,metadata=read_window(args.bag,args.start_ns,args.end_ns,
                                         args.median_imu_ns,args.frames)
    rows=[summarize_config(c,clouds,gravity) for c in configurations()]
    prefix=args.output_prefix
    # Refuse to overwrite any previous sweep evidence.
    csv_path=Path(str(prefix)+'.csv');json_path=Path(str(prefix)+'.json')
    if csv_path.exists() or json_path.exists():raise FileExistsError(prefix)
    with csv_path.open('x',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    with json_path.open('x',encoding='utf-8') as stream:
        json.dump({'schema_version':1,'bag_basename':args.bag.name,
            'start_sensor_ns':args.start_ns,'end_sensor_ns':args.end_ns,
            'median_imu_ns':args.median_imu_ns,'gravity_up':gravity.tolist(),
            'configurations':len(rows),**metadata},stream,sort_keys=True,indent=2)
        stream.write('\n')
    print(json.dumps({'configurations':len(rows),'frames':len(clouds),
                      'csv':str(csv_path),'metadata':str(json_path)},sort_keys=True))


if __name__=='__main__':main()
