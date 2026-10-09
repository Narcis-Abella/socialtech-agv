"""Evaluator-only: score one online run (metrics.json of reloc_node) against the Voxel-SLAM reference. Never mounted in a method container.
Classes follow 02-protocolo-evaluacion.md: correct_accept / wrong_accept (error > 0.30 m or 8 deg) / false_reject_* / correct_reject (negatives)."""
import argparse
import json
from pathlib import Path

import numpy as np

from evaluate import error, reference_at, yaw
from offline_s2 import map_transform


def pose_xyyaw(x, y, w):
    T = np.eye(4)
    T[0, 0], T[0, 1], T[1, 0], T[1, 1] = np.cos(w), -np.sin(w), np.sin(w), np.cos(w)
    T[:2, 3] = [x, y]
    return T


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run-dir', required=True, help='run output dir with work/attempt*/metrics.json and amcl.tum')
    p.add_argument('--reference', required=True)
    p.add_argument('--map-yaml', required=True)
    p.add_argument('--link')
    p.add_argument('--negative', action='store_true', help='no correct pose exists in this map: any acceptance is wrong')
    a = p.parse_args()
    run = Path(a.run_dir)
    files = sorted((run / 'work').glob('attempt*/metrics.json'))
    m = json.loads(files[-1].read_text())
    rows = np.loadtxt(a.reference, usecols=range(8))
    T0 = map_transform(a.map_yaml)
    T_link = np.asarray(json.loads(Path(a.link).read_text())['T_A_B']) if a.link else np.eye(4)
    ref = lambda t: T_link @ T0 @ reference_at(rows, t)
    out = {'run': run.name, 'attempt': m['attempt'], 'state': m['state'], 'reason': m['reason'], 'accepted': m['accepted']}
    ip = m['initialpose']
    if ip:
        xy, deg = error(pose_xyyaw(ip['x'], ip['y'], ip['yaw']), ref(ip['stamp']))
        out['initialpose_error'] = {'xy_m': xy, 'yaw_deg': deg, 'within_limits': xy <= .30 and deg <= 8.}
    if m['accepted']:
        ok = ip and out['initialpose_error']['within_limits'] and not a.negative
        out['class'] = 'correct_accept' if ok else 'wrong_accept'
    else:
        out['class'] = 'correct_reject' if a.negative else ('false_reject_timeout' if m['state'] == 'TIMEOUT' else 'false_reject_' + str(m['reason']))
    tum = run / 'amcl.tum'
    if tum.exists() and tum.stat().st_size:
        A = np.loadtxt(tum, ndmin=2)
        errs = []
        for r in A:
            try:
                T = pose_xyyaw(r[1], r[2], 2 * np.arctan2(r[6], r[7]))
                errs.append((r[0], *error(T, ref(r[0]))))
            except ValueError:
                continue
        if errs:
            e = np.array(errs)
            t0 = ip['stamp'] if ip else e[0, 0]
            late = e[e[:, 0] >= t0 + 5.]
            out['amcl'] = {'n': len(e), 'first_stamp_after_init_s': float(e[0, 0] - t0), 'last_xy_m': float(e[-1, 1]), 'last_yaw_deg': float(e[-1, 2]),
                           'xy_median_m': float(np.median(late[:, 1])) if len(late) else None, 'xy_p95_m': float(np.percentile(late[:, 1], 95)) if len(late) else None,
                           'xy_max_m': float(late[:, 1].max()) if len(late) else None, 'yaw_max_deg': float(late[:, 2].max()) if len(late) else None,
                           'frac_within_limits': float(np.mean((late[:, 1] <= .30) & (late[:, 2] <= 8.))) if len(late) else None}
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
