#!/usr/bin/env python3
"""Read storage/header timestamp pairs for a bounded ROS 2 bag interval.

This is a metadata diagnostic: it deserializes only the selected sensor
messages and never modifies or copies the bag.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def inspect(bag, start_storage_ns, end_storage_ns):
    from rosbags.highlevel import AnyReader

    result={}
    with AnyReader([Path(bag)]) as reader:
        conns=[c for c in reader.connections if c.topic in ('/livox/lidar','/livox/imu')]
        for c,storage_ns,raw in reader.messages(connections=conns,start=start_storage_ns,stop=end_storage_ns):
            msg=reader.deserialize(raw,c.msgtype)
            header_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            result.setdefault(c.topic,[]).append((int(storage_ns),int(header_ns)))
    summary={}
    for topic,rows in sorted(result.items()):
        rows.sort()
        storage=np.asarray([r[0] for r in rows],dtype=np.int64)
        header=np.asarray([r[1] for r in rows],dtype=np.int64)
        delta=np.diff(header)
        pair_offset=header-storage
        summary[topic]={'count':len(rows),'storage_start_ns':int(storage[0]),
            'storage_end_ns':int(storage[-1]),'header_start_ns':int(header[0]),
            'header_end_ns':int(header[-1]),'header_minus_storage_median_ns':int(np.median(pair_offset)),
            'header_minus_storage_p95_abs_ns':int(np.quantile(np.abs(pair_offset),.95)),
            'header_minus_storage_min_ns':int(pair_offset.min()),
            'header_minus_storage_max_ns':int(pair_offset.max()),
            'header_nonpositive_intervals':int((delta<=0).sum()),
            'header_median_delta_ns':float(np.median(delta)) if len(delta) else None,
            'header_max_delta_ns':int(delta.max()) if len(delta) else None,
            'first_pairs_ns':[[int(a),int(b)] for a,b in rows[:3]],
            'last_pairs_ns':[[int(a),int(b)] for a,b in rows[-3:]]}
    return summary


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag',type=Path);ap.add_argument('start_storage_ns',type=int);ap.add_argument('end_storage_ns',type=int)
    a=ap.parse_args()
    if a.end_storage_ns<=a.start_storage_ns:ap.error('end must follow start')
    print(json.dumps(inspect(a.bag,a.start_storage_ns,a.end_storage_ns),sort_keys=True,allow_nan=False))


if __name__=='__main__':main()
