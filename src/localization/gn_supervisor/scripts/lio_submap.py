"""Join body clouds to timestamped odometry in a local first-body gauge, level them by gravity, and write the fit/check submap input of a search.

No reference pose is read. The clouds are the raw Livox frames (reloc_live) or already-deskewed body clouds; the odometry may be an identity stand-in.
"""
import bisect
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from pointcloud import transform, voxel, write_xyz

FIT_FRACTION = 2 / 3   # the first part of an exposure is the fit batch, the rest the check batch (search_v2.temporal_evidence relies on the same split)


def pose(msg):
    p=msg.pose.pose.position
    q=msg.pose.pose.orientation
    T=np.eye(4)
    T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    T[:3,3]=[p.x,p.y,p.z]
    return T


def stamp(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


def interpolate(rows,t):
    # bisect on the stamps (O(log n)); same bracket rule as the former searchsorted version
    i=bisect.bisect_left(rows,t,key=lambda r:r[0])
    if i<len(rows) and abs(rows[i][0]-t)<1e-6:
        return rows[i][1]
    if i==0 or i==len(rows) or rows[i][0]-rows[i-1][0]>.20:
        raise ValueError('no odometry bracket within 0.20 s')
    a,A=rows[i-1]; b,B=rows[i]
    u=(t-a)/(b-a)
    T=np.eye(4)
    T[:3,3]=(1-u)*A[:3,3]+u*B[:3,3]
    T[:3,:3]=Slerp([a,b],Rotation.from_matrix([A[:3,:3],B[:3,:3]]))([t])[0].as_matrix()
    return T


def rebase(points, first, current):
    return transform(points,np.linalg.inv(first)@current)


def level_rotation(up):
    """Smallest rotation taking the measured up direction to +z; adds no yaw."""
    u=np.asarray(up,float); u=u/np.linalg.norm(u)
    z=np.array([0.,0.,1.])
    v=np.cross(u,z); c=float(u@z); s=np.linalg.norm(v)
    if s<1e-12:
        if c>0:
            return np.eye(3)
        raise ValueError('measured up direction points down')
    K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])/s
    a=np.arctan2(s,c)
    return np.eye(3)+np.sin(a)*K+(1-np.cos(a))*K@K


def gravity_up(imu,odom,first,start,end):
    """Mean specific force of the robot's own IMU, rotated into the first local frame.

    Stationary, it is the up direction. While moving, the body accelerations of a
    short start-stop path average out; the residual tilt is reported, not hidden.
    """
    rows=[]
    skipped=0
    for t,a in imu:
        if start<=t<end:
            try:
                R=(np.linalg.inv(first)@interpolate(odom,t))[:3,:3]
            except ValueError:
                skipped+=1  # inside an odometry gap > 0.20 s (e.g. a scan FAST-LIO2 skipped): sample unused
                continue
            rows.append(R@a)
    if len(rows)<20:
        raise ValueError('insufficient IMU samples for levelling')
    rows=np.asarray(rows)
    mean=rows.mean(axis=0)
    spread=float(np.degrees(np.median(np.arccos(np.clip(
        rows@mean/np.linalg.norm(rows,axis=1)/np.linalg.norm(mean),-1,1)))))
    return mean/np.linalg.norm(mean), len(rows), spread, skipped


def cloud_xyz(msg):
    offsets={f.name:f.offset for f in msg.fields}
    data=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(-1,msg.point_step)
    xyz=np.column_stack([data[:,offsets[k]:offsets[k]+4].copy().view('<f4').reshape(-1) for k in 'xyz']).astype(np.float64)
    return xyz[np.isfinite(xyz).all(1)]


def assemble(odom,clouds,imu,map_xyz,start,duration,out,voxel_size=.20,imu_source='/livox/imu',source='online'):
    """Fit/check submap run directory from in-memory data: odom [(t, T)], clouds [(t, xyz)], imu [(t, accel)] or None."""
    out=Path(out); out.mkdir(parents=True,exist_ok=False)
    odom=sorted(odom,key=lambda x:x[0]); clouds=sorted(clouds,key=lambda x:x[0])
    if len(clouds)<3:
        raise ValueError('insufficient FAST-LIO body clouds')
    first=interpolate(odom,clouds[0][0])
    stamps=[t for t,_ in clouds]
    local=[]; counts=[]
    for t,points in clouds:
        local.append(rebase(points,first,interpolate(odom,t)))
        counts.append(len(points))
    T_level=np.eye(4); levelling=None
    if imu is not None:
        up,n,spread,skipped=gravity_up(imu,odom,first,stamps[0],stamps[-1])
        T_level[:3,:3]=level_rotation(up)
        local=[transform(p,T_level) for p in local]
        levelling={'source':imu_source+' mean specific force in first local frame','samples':n,'samples_in_odom_gaps':skipped,
                   'median_sample_angle_to_mean_deg':spread,
                   'tilt_removed_deg':float(np.degrees(np.arccos(np.clip(up[2],-1,1))))}
    fit=[p for t,p in zip(stamps,local) if t<start+duration*FIT_FRACTION]
    check=[p for t,p in zip(stamps,local) if t>=start+duration*FIT_FRACTION]
    if not fit or not check:
        raise ValueError('fit/check batch missing')
    fit=voxel(np.concatenate(fit),voxel_size)
    check=voxel(np.concatenate(check),voxel_size)
    write_xyz(out/'fit.xyz',fit); write_xyz(out/'check.xyz',check)
    shutil.copyfile(map_xyz,out/'map.xyz')
    last=np.linalg.inv(first)@interpolate(odom,stamps[-1])
    manifest={'bag':source,'frame_fit':'local_first_imu_levelled' if imu is not None else 'local_first_imu','frame_map':'map',
              'T_level_local':T_level.tolist(),'levelling':levelling,
              'start_header_s':start,'duration_s':duration,'last_scan_end_s':stamps[-1],
              'stamps_s':stamps,'cloud_points':counts,'fit_points':len(fit),'check_points':len(check),
              'voxel_m':voxel_size,'T_local_imu_at_last':last.tolist(),
              'truth_used':False}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest
