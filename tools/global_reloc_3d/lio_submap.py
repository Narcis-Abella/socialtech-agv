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


def cloud_xyz(msg):
    offsets={f.name:f.offset for f in msg.fields}
    data=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(-1,msg.point_step)
    xyz=np.column_stack([data[:,offsets[k]:offsets[k]+4].copy().view('<f4').reshape(-1) for k in 'xyz']).astype(np.float64)
    return xyz[np.isfinite(xyz).all(1)]


def build(bag,map_xyz,start,duration,out,voxel_size=.20):
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
    stamps=[t for t,_ in clouds]
    local=[]; counts=[]
    for t,points in clouds:
        local.append(rebase(points,first,interpolate(odom,t)))
        counts.append(len(points))
    fit=[p for t,p in zip(stamps,local) if t<start+duration*2/3]
    check=[p for t,p in zip(stamps,local) if t>=start+duration*2/3]
    if not fit or not check:
        raise ValueError('fit/check batch missing')
    fit=voxel(np.concatenate(fit),voxel_size)
    check=voxel(np.concatenate(check),voxel_size)
    write_xyz(out/'fit.xyz',fit); write_xyz(out/'check.xyz',check)
    shutil.copyfile(map_xyz,out/'map.xyz')
    last=np.linalg.inv(first)@interpolate(odom,stamps[-1])
    manifest={'bag':str(bag),'frame_fit':'local_first_imu','frame_map':'map',
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
    a=p.parse_args()
    build(a.bag,a.map_xyz,a.start,a.duration,a.out,a.voxel)
