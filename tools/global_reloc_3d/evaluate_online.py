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


def classify(m, ref, negative, ref_wide=None):
    """Class of one attempt. `ref(t)` returns the reference pose or raises ValueError where there is none.
    A success class is given only with a valid reference at the pose stamp; a pose accepted on a negative is wrong with or without one."""
    out = {'attempt': m['attempt'], 'state': m['state'], 'reason': m['reason'], 'accepted': m['accepted']}
    ip = m['initialpose']
    gap = None
    if ip:
        try:
            xy, deg = error(pose_xyyaw(ip['x'], ip['y'], ip['yaw']), ref(ip['stamp']))
            if not (np.isfinite(xy) and np.isfinite(deg)):
                raise ValueError('pose error not finite')
            out['initialpose_error'] = {'xy_m': xy, 'yaw_deg': deg, 'within_limits': xy <= .30 and deg <= 8.}
        except ValueError as exc:             # the Voxel-SLAM reference has a gap at this stamp: no ground truth, not a method result
            gap = out['reference_gap'] = str(exc)
            # One missing 10 Hz reference sample leaves a 0.2003 s bracket, just over the 0.20 s rule. The class stays
            # 'unscorable' (the rule is not moved), but the pose is still measured across a bracket of up to 0.5 s and labelled so:
            # a published pose must never go without a number only because of that.
            if ref_wide is not None:
                try:
                    xy, deg = error(pose_xyyaw(ip['x'], ip['y'], ip['yaw']), ref_wide(ip['stamp']))
                    if np.isfinite(xy) and np.isfinite(deg):
                        out['initialpose_error_wide_bracket'] = {'xy_m': xy, 'yaw_deg': deg, 'within_limits': xy <= .30 and deg <= 8., 'bracket_s': .5}
                except ValueError:
                    pass
    if ip and not m['accepted']:
        out['class'] = 'pose_without_acceptance'        # must never happen; worse than a wrong accept
    elif m['accepted'] and negative:
        out['class'] = 'wrong_accept'                   # no correct pose exists in this map: wrong whatever the reference says
    elif m['accepted'] and not ip:
        out['class'] = 'accepted_not_published'         # accepted, but the handoff never sent it: nothing to score, not a success
    elif m['accepted'] and gap:
        out['class'] = 'unscorable_reference_gap'
    elif m['accepted']:
        out['class'] = 'correct_accept' if out['initialpose_error']['within_limits'] else 'wrong_accept'
    elif m['state'] == 'ERROR':
        out['class'] = 'error_no_pose'      # nothing was published; neither a correct reject nor a method rejection
    else:
        out['class'] = 'correct_reject' if negative else ('false_reject_timeout' if m['state'] == 'TIMEOUT' else 'false_reject_' + str(m['reason']))
    return out


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
    out = {'run': run.name, **classify(m, ref, a.negative, lambda t: T_link @ T0 @ reference_at(rows, t, .5))}
    ip = m['initialpose']
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
        out['amcl_poses'] = {'n': len(A), 'unscorable_reference_gap': len(A) - len(errs)}
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
