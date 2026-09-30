"""Replace KISS-ICP pose times (bag time) with the scans' header.stamp. usage: restamp.py <bag> <topic> <in.tum> <out.tum>"""
import sys
from pathlib import Path
from rosbags.highlevel import AnyReader

bag, topic, src, dst = sys.argv[1:5]
with AnyReader([Path(bag)]) as r:
    st = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
          for c, _, raw in r.messages(connections=[x for x in r.connections if x.topic == topic])
          for m in [r.deserialize(raw, c.msgtype)]]
rows = [l.split(maxsplit=1) for l in open(src)]
assert len(rows) == len(st), (len(rows), len(st))
open(dst, "w").writelines(f"{t:.9f} {rest}" for t, (_, rest) in zip(st, rows))
