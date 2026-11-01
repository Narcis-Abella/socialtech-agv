"""Read-only raw sensor timing/IMU index. No reference access, ROS or registration.

Run inside an existing environment with numpy and rosbags:
  python3 -B window_sensor_index.py BAG OUT
OUT must be a new directory. Large bag payloads are never hashed or copied.
"""
import argparse
import gzip
import hashlib
import importlib.metadata
import json
from pathlib import Path
import struct
import numpy as np


def header(raw):
    if bytes(raw[:4]) != b'\x00\x01\x00\x00':
        raise ValueError('Only little-endian CDR1 supported')
    sec,nsec,n=struct.unpack_from('<iII',raw,4)
    if nsec>=1000000000 or n<1 or len(raw)<16+n or raw[15+n]!=0:
        raise ValueError('Invalid header')
    return sec*1000000000+nsec,bytes(raw[16:15+n]).decode(),16+n


def custom_timing(raw):
    stamp,frame,off=header(raw)
    off+=(-(off-4))%8
    tb,num=struct.unpack_from('<QI',raw,off)
    count=struct.unpack_from('<I',raw,off+16)[0]
    off+=20
    if count!=num or count==0 or len(raw) not in (off+20*count,off+20*count-1):
        raise ValueError('Invalid CustomMsg point count/length')
    offsets=np.ndarray((count,),dtype='<u4',buffer=raw,offset=off,strides=(20,))
    return [stamp,int(tb),int(offsets.min()),int(offsets.max()),int(offsets[-1]),int(count)],frame


def imu_values(raw):
    stamp,frame,off=header(raw)
    off+=(-(off-4))%8
    values=struct.unpack_from('<37d',raw,off)
    if len(raw)!=off+37*8: raise ValueError('Invalid Imu length')
    # quaternion 4, orientation covariance 9, gyro 3, covariance 9, accel 3, covariance 9
    v=list(values[13:16])+list(values[25:28])
    if not np.isfinite(v).all(): raise ValueError('Nonfinite IMU')
    return stamp,v,frame


def main():
    from rosbags.highlevel import AnyReader
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag',type=Path); ap.add_argument('out',type=Path)
    a=ap.parse_args(); a.out.mkdir(parents=True,exist_ok=False)
    result={'schema':1,'lidar':[],'imu':[],'frame_ids':{},'rosbags_version':importlib.metadata.version('rosbags'),
            'reader_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'sources':[],'bag_digest_status':'not_computed','bag_sha256':None}
    for p in sorted(a.bag.iterdir()):
        if p.is_file():
            st=p.stat(); entry={'name':p.name,'size':st.st_size,'mtime_ns':st.st_mtime_ns}
            if p.name=='metadata.yaml': entry['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
            result['sources'].append(entry)
    with AnyReader([a.bag]) as reader:
        connections=[c for c in reader.connections if c.topic in ('/livox/lidar','/livox/imu')]
        expected={'/livox/lidar':'livox_ros_driver2/msg/CustomMsg','/livox/imu':'sensor_msgs/msg/Imu'}
        if len(connections)!=2 or {c.topic:c.msgtype for c in connections}!=expected: raise ValueError('Unexpected connections')
        result['topics']=[{'topic':c.topic,'type':c.msgtype,'count':c.msgcount} for c in connections]
        seen=set()
        for c,t,raw in reader.messages(connections=connections):
            if c.topic.endswith('lidar'):
                vals,frame=custom_timing(raw)
                result['lidar'].append([int(t),*vals])
            else:
                stamp,vals,frame=imu_values(raw)
                result['imu'].append([int(t),stamp,*vals])
            result['frame_ids'][c.topic]=frame
            if c.topic not in seen:
                # Independent schema decoder validates the fast CDR decoder once per topic.
                msg=reader.deserialize(raw,c.msgtype)
                assert msg.header.stamp.sec*10**9+msg.header.stamp.nanosec==(vals[0] if c.topic.endswith('lidar') else stamp)
                if c.topic.endswith('lidar'):
                    assert msg.timebase==vals[1] and len(msg.points)==vals[-1]
                    assert msg.points[-1].offset_time==vals[4]
                else:
                    assert [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,
                            msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]==vals
                seen.add(c.topic)
    for c in result['topics']:
        if len(result['lidar' if c['topic'].endswith('lidar') else 'imu'])!=c['count']: raise ValueError('Message count mismatch')
    data=json.dumps(result,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
    (a.out/'sensor-index.json.gz').write_bytes(gzip.compress(data,mtime=0))
    print(json.dumps({'lidar':len(result['lidar']),'imu':len(result['imu']),'index_sha256':hashlib.sha256(data).hexdigest()}))

if __name__=='__main__': main()
