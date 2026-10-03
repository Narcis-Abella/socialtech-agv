"""Mocap PoseStamped -> TUM. usage: gt_from_bag.py <bag_dir> <topic> <out.tum>"""
import sys
from pathlib import Path
from rosbags.highlevel import AnyReader

bag, topic, out = sys.argv[1:4]
with AnyReader([Path(bag)]) as r, open(out, "w") as f:
    for c, _, raw in r.messages(connections=[x for x in r.connections if x.topic == topic]):
        m = r.deserialize(raw, c.msgtype)
        p, q, s = m.pose.position, m.pose.orientation, m.header.stamp
        f.write(f"{s.sec + s.nanosec * 1e-9:.9f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")
