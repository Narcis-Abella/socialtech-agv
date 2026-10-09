"""Join FAST-LIO2 body clouds to timestamped odometry in a local first-body gauge.

No prior map or reference is read. Body clouds are already deskewed by FAST-LIO2.
"""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from offline_s2 import transform, voxel, write_xyz


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
    ts=np.array([r[0] for r in rows])
    i=np.searchsorted(ts,t)
    if i<len(rows) and abs(rows[i][0]-t)<1e-6:
        return rows[i][1]
    if i==0 or i==len(rows) or ts[i]-ts[i-1]>.20:
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


def read_imu(bag,start,end):
    from rosbags.highlevel import AnyReader
    out=[]
    with AnyReader([Path(bag)]) as reader:
        conn=[c for c in reader.connections if c.topic=='/livox/imu']
        if len(conn)!=1:
            raise ValueError('expected exactly one /livox/imu connection')
        for c,_,raw in reader.messages(connections=conn,start=int((start-1)*1e9),stop=int((end+1)*1e9)):
            msg=reader.deserialize(raw,c.msgtype)
            a=msg.linear_acceleration
            out.append((stamp(msg),np.array([a.x,a.y,a.z])))
    return out


def s3_limit(stamps,poses,max_radius,max_path):
    """Number of leading clouds inside the online S3 bounds, from own odometry only."""
    xy=np.array([T[:2,3] for T in poses])
    radius=np.linalg.norm(xy-xy[0],axis=1)
    path=np.concatenate([[0.],np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))])
    over=np.flatnonzero((radius>max_radius)|(path>max_path))
    n=int(over[0]) if len(over) else len(stamps)
    reason='radius' if len(over) and radius[n]>max_radius else 'path' if len(over) else 'duration'
    return n,reason,float(path[n-1]),float(radius[:n].max())


def build(bag,map_xyz,start,duration,out,voxel_size=.20,imu_bag=None,max_radius=None,max_path=None):
    from rosbags.highlevel import AnyReader
    out=Path(out); out.mkdir(parents=True,exist_ok=False)
    odom=[]; clouds=[]
    with AnyReader([Path(bag)]) as reader:
        conn=[c for c in reader.connections if c.topic in ('/Odometry','/cloud_registered_body')]
        for c,_,raw in reader.messages(connections=conn):
            msg=reader.deserialize(raw,c.msgtype)
            t=stamp(msg)
            if c.topic=='/Odometry':
                odom.append((t,pose(msg)))
            elif start <= t < start+duration:
                if msg.header.frame_id!='body':
                    raise ValueError('FAST-LIO cloud frame is not body')
                clouds.append((t,cloud_xyz(msg)))
    odom.sort(key=lambda x:x[0]); clouds.sort(key=lambda x:x[0])
    if len(clouds)<3:
        raise ValueError('insufficient FAST-LIO body clouds')
    first=interpolate(odom,clouds[0][0])
    stop=None
    if max_radius is not None:
        rel=[np.linalg.inv(first)@interpolate(odom,t) for t,_ in clouds]
        n,reason,path,radius=s3_limit([t for t,_ in clouds],rel,max_radius,max_path)
        clouds=clouds[:n]
        # The fit/check split and temporal rule use the span actually kept.
        duration=min(duration,clouds[-1][0]-start+.1)
        stop={'reason':reason,'clouds_kept':n,'odom_path_m':path,'odom_max_radius_m':radius,
              'max_radius_m':max_radius,'max_path_m':max_path}
    stamps=[t for t,_ in clouds]
    local=[]; counts=[]
    for t,points in clouds:
        local.append(rebase(points,first,interpolate(odom,t)))
        counts.append(len(points))
    T_level=np.eye(4); levelling=None
    if imu_bag:
        up,n,spread,skipped=gravity_up(read_imu(imu_bag,start,start+duration),odom,first,stamps[0],stamps[-1])
        T_level[:3,:3]=level_rotation(up)
        local=[transform(p,T_level) for p in local]
        levelling={'source':'/livox/imu mean specific force in first local frame','samples':n,'samples_in_odom_gaps':skipped,
                   'median_sample_angle_to_mean_deg':spread,
                   'tilt_removed_deg':float(np.degrees(np.arccos(np.clip(up[2],-1,1))))}
    fit=[p for t,p in zip(stamps,local) if t<start+duration*2/3]
    check=[p for t,p in zip(stamps,local) if t>=start+duration*2/3]
    if not fit or not check:
        raise ValueError('fit/check batch missing')
    fit=voxel(np.concatenate(fit),voxel_size)
    check=voxel(np.concatenate(check),voxel_size)
    write_xyz(out/'fit.xyz',fit); write_xyz(out/'check.xyz',check)
    shutil.copyfile(map_xyz,out/'map.xyz')
    last=np.linalg.inv(first)@interpolate(odom,stamps[-1])
    manifest={'bag':str(bag),'frame_fit':'local_first_imu_levelled' if imu_bag else 'local_first_imu','frame_map':'map',
              'T_level_local':T_level.tolist(),'levelling':levelling,'s3_stop':stop,
              'start_header_s':start,'duration_s':duration,'last_scan_end_s':stamps[-1],
              'stamps_s':stamps,'cloud_points':counts,'fit_points':len(fit),'check_points':len(check),
              'voxel_m':voxel_size,'T_local_imu_at_last':last.tolist(),
              'truth_used':False,'frontend':'FAST_LIO2_body_cloud_joined_to_Odometry'}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--bag',required=True)
    p.add_argument('--map-xyz',required=True)
    p.add_argument('--start',type=float,required=True)
    p.add_argument('--duration',type=float,default=3.)
    p.add_argument('--out',required=True)
    p.add_argument('--voxel',type=float,default=.20)
    p.add_argument('--imu-bag',help='raw sensor bag with /livox/imu; levels the local frame by gravity')
    p.add_argument('--max-radius',type=float,help='online S3 stop radius from own odometry (2.8)')
    p.add_argument('--max-path',type=float,default=6.)
    a=p.parse_args()
    build(a.bag,a.map_xyz,a.start,a.duration,a.out,a.voxel,a.imu_bag,a.max_radius,a.max_path)
