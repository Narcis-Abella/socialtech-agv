"""Evaluator-only: for how long was the gated output non-zero while AMCL's pose was outside 0.30 m / 8 deg of the reference?
Answers "how late is the revocation" with the reference, not with the node's own monitor. Never mounted in a method container.

  permit_exposure.py --run-dir RUN --reference alidarState.txt --map-yaml map.yaml
Clock alignment (1x replay): bag time = initialpose stamp + its age at publication + (monotonic - monotonic at publication).

Timeline fields (seconds after LOCALIZED, node monotonic clock; AMCL poses arrive every ~0.2 s, which is the resolution):
  out_at_s      start of the run of outside poses that is still going on when the gate closes for the last time (None: inside at closing)
  close_at_s    last instant of non-zero gated output
  latency_s     close_at_s - out_at_s: how long the output stayed non-zero on a pose that had left the limits (0 if out_at_s is None)
  lead_s        gate closed first: time from closing to the first outside pose afterwards (None if it never leaves)
  first_failed_check_s  [newest scan, verdict] of the first failed map check, from supervision.json"""
import argparse
import json
from pathlib import Path

import numpy as np

from evaluate import error, reference_at
from evaluate_online import pose_xyyaw
from offline_s2 import map_transform


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run-dir', required=True)
    p.add_argument('--reference', required=True)
    p.add_argument('--map-yaml', required=True)
    a = p.parse_args()
    run = Path(a.run_dir)
    mp = sorted((run / 'work').glob('attempt*/metrics.json'))[-1]
    m, m_dir = json.loads(mp.read_text()), mp.parent.name
    out = {'run': run.name, 'state': m['state'], 'open_s': 0., 'scored_s': 0., 'outside_s': 0., 'max_xy_m': None, 'max_yaw_deg': None}
    ip, probe = m.get('initialpose'), run / 'gate_probe.json'
    if ip and probe.exists() and (run / 'amcl.tum').stat().st_size:
        pub_mono = m['first_input_mono'] + m['marks_rel_first_input_s']['initialpose_published']
        to_bag = lambda mono: ip['stamp'] + ip.get('map_check_age_s', 0.) + mono - pub_mono
        rows = np.loadtxt(a.reference, usecols=range(8))
        T0 = map_transform(a.map_yaml)
        A = np.loadtxt(run / 'amcl.tum', ndmin=2)
        open_bag = [(to_bag(s), to_bag(e)) for s, e in json.loads(probe.read_text())['nonzero_intervals']]
        out['open_s'] = float(sum(e - s for s, e in open_bag))
        worst, run_out, last = [], None, None      # run_out: (start, worst xy, worst deg) of the current run of outside poses while open
        to_mono = lambda bag: bag - ip['stamp'] - ip.get('map_check_age_s', 0.) + pub_mono
        loc = m['first_input_mono'] + m['marks_rel_first_input_s'].get('state:LOCALIZED', m['marks_rel_first_input_s']['initialpose_published'])
        for r, t_next in zip(A, list(A[1:, 0]) + [A[-1, 0]]):
            dt = sum(max(0., min(e, t_next) - max(s, r[0])) for s, e in open_bag)   # this pose is the newest one for dt of open gate
            if dt <= 0.:
                continue
            try:
                xy, deg = error(pose_xyyaw(r[1], r[2], 2 * np.arctan2(r[6], r[7])), T0 @ reference_at(rows, r[0]))
            except ValueError:
                continue
            out['scored_s'] += dt
            if xy > .30 or deg > 8.:
                out['outside_s'] += dt
                start = max(r[0], min(s for s, e in open_bag if e > r[0]))
                run_out = (start, xy, deg) if run_out is None else (run_out[0], max(run_out[1], xy), max(run_out[2], deg))
            else:
                run_out = None
            worst.append((xy, deg))
        if open_bag:
            close = max(e for s, e in open_bag)
            out.update(close_at_s=to_mono(close) - loc, out_at_s=None, latency_s=0., lead_s=None)
            if run_out:
                out.update(out_at_s=to_mono(run_out[0]) - loc, latency_s=close - run_out[0], exposure_max_xy_m=run_out[1], exposure_max_yaw_deg=run_out[2])
            else:
                for r in A[A[:, 0] >= close]:
                    try:
                        xy, deg = error(pose_xyyaw(r[1], r[2], 2 * np.arctan2(r[6], r[7])), T0 @ reference_at(rows, r[0]))
                    except ValueError:
                        continue
                    if xy > .30 or deg > 8.:
                        out['lead_s'] = float(r[0] - close)
                        break
        sup = run / 'work' / m_dir / 'supervision.json'
        if sup.exists():
            ms = json.loads(sup.read_text()).get('map_supervision') or {}
            tr = ms.get('trace_t_ok_reason_support_rmse_shiftxy_shiftyaw_wall') or []
            bad = next((t for t in tr if not t[1]), None)
            out.update(checks=ms.get('n'), checks_failed=ms.get('failed'), first_failed_check_s=bad and [bad[0], round(bad[0] + bad[7], 2)],
                       check_wall_s_median=float(np.median([t[7] for t in tr])) if tr else None)
        if worst:
            out['max_xy_m'], out['max_yaw_deg'] = float(max(w[0] for w in worst)), float(max(w[1] for w in worst))
    print(json.dumps({k: round(v, 3) if isinstance(v, float) else v for k, v in out.items()}))


if __name__ == '__main__':
    main()
