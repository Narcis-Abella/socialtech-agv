#!/usr/bin/env python3
"""Estimate sensor-to-floor height from a Livox bag, IMU gravity and LiDAR only.

No map, trajectory, Voxel-SLAM reference or evaluator inventory is read. This is
an exploratory geometric estimator, not a certified mechanical calibration.
Requires numpy and rosbags (available in the existing mapper image).
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


VERSION='0.1'


def gravity_basis(up):
    up=np.asarray(up,dtype=float)
    norm=float(np.linalg.norm(up))
    if up.shape!=(3,) or not np.isfinite(up).all() or norm<1e-8:
        raise ValueError('gravity vector must be finite and nonzero')
    up=up/norm
    ref=np.array([1.,0.,0.]) if abs(up[0])<.9 else np.array([0.,1.,0.])
    u=np.cross(ref,up);u/=np.linalg.norm(u)
    v=np.cross(up,u)
    return u,v,up


def _failure(reason,n_points=0,n_down=0):
    return {'supported':False,'reason':reason,'point_count':int(n_points),
            'downward_point_count':int(n_down),'inliers':0,'inlier_cells':0,
            'inlier_ratio':0.,'height_m':None,'tilt_deg':None,
            'residual_p95_m':None,'horizontal_span_m':None}


def estimate_ground_plane(points_xyz,gravity_up,*,cell_size_m=.25,plane_tolerance_m=.03,
                          min_inliers=100,min_inlier_ratio=.10,min_cells=8,min_span_m=.5,
                          max_tilt_deg=10.,iterations=1200,seed=0):
    """Fit the lowest broad downward horizontal plane with deterministic RANSAC.

    Thresholds are exploratory estimator guards and deliberately do not alter
    point-4 stationarity or localization criteria.
    """
    points=np.asarray(points_xyz,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():
        raise ValueError('points must be finite Nx3')
    if cell_size_m<=0 or plane_tolerance_m<=0 or iterations<1:
        raise ValueError('invalid estimator parameters')
    u,v,up=gravity_basis(gravity_up)
    local=np.column_stack((points@u,points@v,points@up))
    downward=local[:,2]<-.05
    below=local[downward]
    if len(below)<min_inliers:
        return _failure('no_downward_returns',len(points),len(below))
    if min(float(np.ptp(below[:,0])),float(np.ptp(below[:,1])))<min_span_m:
        return _failure('insufficient_horizontal_spread',len(points),len(below))
    ix=np.floor((below[:,0]-below[:,0].min())/cell_size_m).astype(np.int64)
    iy=np.floor((below[:,1]-below[:,1].min())/cell_size_m).astype(np.int64)
    lowest={}
    for x,y,z in zip(ix,iy,below[:,2]):
        key=(int(x),int(y))
        if key not in lowest or z<lowest[key]:
            lowest[key]=float(z)
    cells=np.asarray([(x,y,z) for (x,y),z in sorted(lowest.items())],dtype=float)
    if len(cells)<3:
        return _failure('insufficient_plane_cells',len(points),len(below))
    # Distinct-cell minima yield candidate planes while a vertical wall has
    # little horizontal area. The original points determine final support.
    cell_xy=np.column_stack((below[:,0].min()+cells[:,0]*cell_size_m+cell_size_m/2,
                             below[:,1].min()+cells[:,1]*cell_size_m+cell_size_m/2))
    design=np.column_stack((cell_xy,np.ones(len(cell_xy))))
    rng=np.random.default_rng(seed)
    best=None;saw_support=False;saw_horizontal=False
    for sample in rng.integers(0,len(cells),size=(iterations,3)):
        if len(set(map(int,sample)))<3:
            continue
        a=design[sample]
        if abs(np.linalg.det(a))<1e-8:
            continue
        coeff=np.linalg.solve(a,cells[sample,2])
        slope=math.hypot(float(coeff[0]),float(coeff[1]))
        tilt=math.degrees(math.atan(slope))
        if tilt>max_tilt_deg:
            continue
        residual=np.abs(below[:,2]-(coeff[0]*below[:,0]+coeff[1]*below[:,1]+coeff[2]))
        inlier=residual<=plane_tolerance_m
        count=int(inlier.sum())
        if count<min_inliers or count/len(below)<min_inlier_ratio:
            continue
        saw_support=True
        binx=np.floor((below[inlier,0]-below[:,0].min())/cell_size_m).astype(int)
        biny=np.floor((below[inlier,1]-below[:,1].min())/cell_size_m).astype(int)
        inlier_cells=len(set(zip(binx.tolist(),biny.tolist())))
        if inlier_cells<min_cells:
            continue
        span=float(min(np.ptp(below[inlier,0]),np.ptp(below[inlier,1])))
        if span<min_span_m:
            continue
        saw_horizontal=True
        p95=float(np.quantile(residual[inlier],.95))
        # Prefer the broadest supported horizontal return surface. This may be
        # a robot/platform surface rather than the physical floor, so a stable
        # result is explicitly only a plane candidate, never height calibration.
        score=(count,inlier_cells,-p95,-float(coeff[2]),tuple(-np.round(coeff,9)))
        if best is None or score>best[0]:
            best=(score,coeff,count,inlier_cells,span,p95,tilt)
    if best is None:
        reason='insufficient_horizontal_spread' if saw_support and not saw_horizontal else 'insufficient_plane_support'
        return _failure(reason,len(points),len(below))
    _,coeff,count,inlier_cells,span,p95,tilt=best
    height=-float(coeff[2])/math.sqrt(1.+float(coeff[0])**2+float(coeff[1])**2)
    if height<=0:
        return _failure('plane_not_below_sensor',len(points),len(below))
    return {'supported':True,'reason':'supported_plane','point_count':int(len(points)),
            'downward_point_count':int(len(below)),'inliers':int(count),
            'inlier_cells':int(inlier_cells),'inlier_ratio':float(count/len(below)),
            'height_m':height,'tilt_deg':float(tilt),'residual_p95_m':p95,
            'horizontal_span_m':span,'plane_coeff_local':[float(x) for x in coeff]}


def summarize_frames(frames,*,minimum_valid_frames=3,stable_p95_p05_m=.10):
    valid=[f for f in frames if f.get('supported') and f.get('height_m') is not None]
    if not valid:
        return {'status':'no_supported_ground_plane','frame_count':len(frames),'valid_frames':0,
                'height_median':None,'height_p05':None,'height_p95':None,'height_std':None,'stable':False}
    heights=np.asarray([f['height_m'] for f in valid],dtype=float)
    p05,p95=(float(x) for x in np.quantile(heights,[.05,.95]))
    stable=(len(valid)>=minimum_valid_frames and p95-p05<=stable_p95_p05_m)
    return {'status':'stable_plane_candidate_NOT_floor_verified' if stable else 'low_confidence_plane_candidate',
            'frame_count':len(frames),'valid_frames':len(valid),
            'height_median':float(np.median(heights)),'height_p05':p05,'height_p95':p95,
            'height_std':float(np.std(heights,ddof=1)) if len(heights)>1 else 0.,
            'stable':bool(stable),'stability_span_p95_p05_m':p95-p05,
            'total_inliers':int(sum(f['inliers'] for f in valid)),
            'median_inlier_ratio':float(np.median([f['inlier_ratio'] for f in valid])),
            'median_tilt_deg':float(np.median([f['tilt_deg'] for f in valid]))}


def estimate_bag_interval(bag,topic_lidar,topic_imu,start_ns,end_ns,*,max_frames=100,seed=0):
    """Read one half-open sensor-time interval from a bag using header stamps."""
    from rosbags.highlevel import AnyReader
    reader=AnyReader([Path(bag)])
    with reader:
        cs={c.topic:c for c in reader.connections if c.topic in (topic_lidar,topic_imu)}
        if set(cs)!={topic_lidar,topic_imu}:
            raise ValueError('required LiDAR and IMU topics not both present')
        if cs[topic_lidar].msgtype!='livox_ros_driver2/msg/CustomMsg' or cs[topic_imu].msgtype!='sensor_msgs/msg/Imu':
            raise ValueError('unexpected sensor message types')
        # Bag storage timestamps differ slightly from sensor headers; read a
        # broad 0.5 s guard, then select strictly by original header stamps.
        start_storage=max(0,start_ns-500_000_000);stop_storage=end_ns+500_000_000
        messages=[]
        for c,storage_ns,raw in reader.messages(connections=list(cs.values()),start=start_storage,stop=stop_storage):
            m=reader.deserialize(raw,c.msgtype)
            h=m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
            if not start_ns<=h<end_ns:
                continue
            frame=m.header.frame_id
            if c.topic==topic_imu:
                a=m.linear_acceleration
                messages.append(('imu',int(h),frame,np.array([a.x,a.y,a.z],dtype=float)))
            else:
                xyz=np.asarray([(p.x,p.y,p.z) for p in m.points],dtype=float)
                messages.append(('lidar',int(h),frame,xyz))
        imu=[x for x in messages if x[0]=='imu']; lidar=[x for x in messages if x[0]=='lidar']
        if not imu or not lidar:
            raise ValueError('no paired sensor messages in interval')
        imu_frames={x[2] for x in imu};lidar_frames={x[2] for x in lidar}
        if len(imu_frames)!=1 or imu_frames!=lidar_frames:
            raise ValueError('LiDAR and IMU frame IDs differ; explicit independent rotation required')
        imu_t=np.asarray([x[1] for x in imu],dtype=np.int64)
        imu_a=np.asarray([x[3] for x in imu],dtype=float)
        frame_results=[]
        # Select evenly across the interval, independent of any reference pose.
        indices=np.unique(np.linspace(0,len(lidar)-1,min(max_frames,len(lidar)),dtype=int))
        for i in indices:
            kind,stamp,_,xyz=lidar[int(i)]
            near=np.abs(imu_t-stamp)<=50_000_000
            if int(near.sum())<5:
                frame_results.append({'supported':False,'reason':'insufficient_nearby_imu','point_count':len(xyz),'inliers':0})
                continue
            accel=np.median(imu_a[near],axis=0)
            frame_results.append(estimate_ground_plane(xyz,accel,seed=seed))
        return {'schema_version':1,'estimator_version':VERSION,'bag_basename':Path(bag).name,
                'topic_lidar':topic_lidar,'topic_imu':topic_imu,'start_sensor_ns':int(start_ns),
                'end_sensor_ns':int(end_ns),'lidar_frame_id':next(iter(lidar_frames)),
                'imu_frame_id':next(iter(imu_frames)),'lidar_messages_in_interval':len(lidar),
                'imu_messages_in_interval':len(imu),'selected_frames':len(indices),
                'frame_estimates':frame_results,'summary':summarize_frames(frame_results)}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag',type=Path);ap.add_argument('start_ns',type=int);ap.add_argument('end_ns',type=int)
    ap.add_argument('--lidar-topic',default='/livox/lidar');ap.add_argument('--imu-topic',default='/livox/imu')
    ap.add_argument('--max-frames',type=int,default=100);ap.add_argument('--seed',type=int,default=0)
    a=ap.parse_args()
    if a.end_ns<=a.start_ns or a.max_frames<1:ap.error('invalid interval or max-frames')
    print(json.dumps(estimate_bag_interval(a.bag,a.lidar_topic,a.imu_topic,a.start_ns,a.end_ns,
          max_frames=a.max_frames,seed=a.seed),sort_keys=True,allow_nan=False))


if __name__=='__main__':main()
