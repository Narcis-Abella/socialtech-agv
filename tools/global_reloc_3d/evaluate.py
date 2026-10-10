"""Post-registration evaluator. Keep this file and its inputs out of method containers."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from offline_s2 import map_transform


def pose(p, q):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q).as_matrix()
    T[:3, 3] = p
    return T


def reference_at(rows, t, max_gap=.20):
    i = np.searchsorted(rows[:, 0], t)
    if i == 0 or i == len(rows) or rows[i, 0]-rows[i-1, 0] > max_gap:
        raise ValueError(f'reference timestamp not bracketed within {max_gap:.2f} s')
    a, b = rows[i-1], rows[i]
    if not (np.isfinite(t) and np.isfinite(a).all() and np.isfinite(b).all()):
        raise ValueError('reference not finite at this timestamp')
    u = (t-a[0])/(b[0]-a[0])
    rotation = Slerp([a[0], b[0]], Rotation.from_quat([a[4:8], b[4:8]]))([t])[0]
    return pose((1-u)*a[1:4]+u*b[1:4], rotation.as_quat())


def yaw(T):
    return np.arctan2(T[1, 0], T[0, 0])


def error(T_est, T_ref):
    xy = float(np.linalg.norm(T_est[:2, 3]-T_ref[:2, 3]))
    d = yaw(T_est)-yaw(T_ref)
    deg = float(np.degrees(abs(np.arctan2(np.sin(d), np.cos(d)))))
    return xy, deg


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', required=True)
    p.add_argument('--search', help='versioned tile-search output; evaluates its selected hypothesis')
    p.add_argument('--reference', required=True)
    p.add_argument('--map-yaml', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--link', help='evaluator frame link JSON T_A_B: reference session map B -> known map A')
    a = p.parse_args()
    run = Path(a.run)
    manifest = json.loads((run/'manifest.json').read_text())
    if a.search:
        search = Path(a.search)
        outcome = json.loads((search/'search.json').read_text())
        result = json.loads((search/outcome['primary_id']/'result.json').read_text())
        accepted = outcome['accepted']
    else:
        result = json.loads((run/'result.json').read_text())
        accepted = json.loads((run/'verification.json').read_text())['accepted']
    t = manifest['last_scan_end_s']
    T_ref = map_transform(a.map_yaml) @ reference_at(np.loadtxt(a.reference, usecols=range(8)), t)
    if a.link:
        T_ref = np.asarray(json.loads(Path(a.link).read_text())['T_A_B']) @ T_ref
    # mid360.yaml/map_visibility.py fixed LiDAR origin in IMU body, R=I.
    T_imu_lidar = np.eye(4)
    T_imu_lidar[:3, 3] = [-.011, -.02329, .04412]
    output = {'represented_time_s': t, 'accepted': accepted, 'link': a.link,
              'reference_kind': 'other_session_Voxel_SLAM_IMU_pose_via_map_link' if a.link else 'same_run_Voxel_SLAM_IMU_pose', 'T_map_imu_reference': T_ref.tolist()}
    for name in ['kiss', 'refined']:
        T_global_local = np.asarray(result['T_map_lidar_'+name])
        if manifest['frame_fit'] == 'local_first_imu':
            T_est = T_global_local @ np.asarray(manifest['T_local_imu_at_last'])
        elif manifest['frame_fit'] == 'local_first_imu_levelled':
            T_est = T_global_local @ np.asarray(manifest['T_level_local']) @ np.asarray(manifest['T_local_imu_at_last'])
        elif manifest['frame_fit'] == 'livox_lidar_stationary':
            T_est = T_global_local @ np.linalg.inv(T_imu_lidar)
        else:
            raise ValueError('unsupported source frame')
        xy, deg = error(T_est, T_ref)
        output[name] = {'xy_error_m': xy, 'yaw_error_deg': deg,
                        'within_limits': xy <= .30 and deg <= 8., 'T_map_imu_estimate': T_est.tolist()}
    output['class'] = ('correct_accept' if output['refined']['within_limits'] else 'wrong_accept') if accepted else 'diagnostic_correct_not_accepted' if output['refined']['within_limits'] else 'diagnostic_wrong_not_accepted'
    Path(a.out).write_text(json.dumps(output, indent=2)+'\n')
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
