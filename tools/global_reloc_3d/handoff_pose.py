"""Truth-free handoff of an accepted global pose to the nav bench (AMCL).

Frames: FAST-LIO2's camera_init (ci) is the IMU frame at its start; the
relocalizer's source frame is the levelled first-cloud frame:
p_src = R_level @ inv(T_ci_first) @ p_ci. Hence
  T_map_ci = T_map_src @ T_level_local @ inv(T_ci_first)
  LEVEL_Q  = rotation of T_level_local @ inv(T_ci_first) (gravity only; odom keeps
             camera_init's yaw, AMCL estimates map->odom)
  INIT_XYYAW = x, y, yaw of T_map_ci: the IMU origin at FAST-LIO2 start, used as the
             base_footprint proxy (sensor->base extrinsic not calibrated: NV-P).
Valid when the bench replays from the same bag offset as the capture, so its
FAST-LIO2 restarts on the same data (certified stationary start).
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from lio_submap import interpolate, pose, stamp


def first_cloud_pose(capture_bag, first_stamp):
    from rosbags.highlevel import AnyReader
    odom = []
    with AnyReader([Path(capture_bag)]) as reader:
        conn = [c for c in reader.connections if c.topic == '/Odometry']
        for c, _, raw in reader.messages(connections=conn):
            m = reader.deserialize(raw, c.msgtype)
            odom.append((stamp(m), pose(m)))
    odom.sort(key=lambda x: x[0])
    return interpolate(odom, first_stamp)


def handoff(T_map_src, T_level_local, T_ci_first):
    T_src_ci = T_level_local @ np.linalg.inv(T_ci_first)
    T_map_ci = T_map_src @ T_src_ci
    q = Rotation.from_matrix(T_src_ci[:3, :3]).as_quat()
    return T_map_ci, q


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', required=True, help='method input (manifest)')
    p.add_argument('--search', required=True, help='frozen accepted search output')
    p.add_argument('--capture', required=True, help='LIO capture bag of the same run')
    p.add_argument('--out', required=True)
    a = p.parse_args()
    m = json.loads((Path(a.run) / 'manifest.json').read_text())
    s = json.loads((Path(a.search) / 'search.json').read_text())
    if not s['accepted']:
        raise SystemExit('refusing handoff: search did not accept')
    T_map_src = np.asarray(json.loads((Path(a.search) / s['primary_id'] / 'result.json').read_text())['T_map_lidar_refined'])
    T_ci_first = first_cloud_pose(a.capture, m['stamps_s'][0])
    T_map_ci, q = handoff(T_map_src, np.asarray(m['T_level_local']), T_ci_first)
    yaw = math.atan2(T_map_ci[1, 0], T_map_ci[0, 0])
    out = {'INIT_XYYAW': f'{T_map_ci[0, 3]:.4f} {T_map_ci[1, 3]:.4f} {yaw:.6f}',
           'LEVEL_Q': ' '.join(f'{v:.9f}' for v in q), 'T_map_ci': T_map_ci.tolist(),
           'T_ci_first': T_ci_first.tolist(), 'first_cloud_stamp_s': m['stamps_s'][0], 'truth_used': False}
    Path(a.out).write_text(json.dumps(out, indent=2) + '\n')
    print(out['INIT_XYYAW'], '|', out['LEVEL_Q'])


if __name__ == '__main__':
    main()
