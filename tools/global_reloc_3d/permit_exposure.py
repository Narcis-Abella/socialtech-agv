"""Evaluator-only: for how long was the gated output non-zero while AMCL's pose was outside 0.30 m / 8 deg of the reference?
Answers "how late is the revocation" with the reference, not with the node's own monitor. Never mounted in a method container.

  permit_exposure.py --run-dir RUN --reference alidarState.txt --map-yaml map.yaml
Clock alignment (1x replay): bag time = initialpose stamp + its age at publication + (monotonic - monotonic at publication)."""
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
    m = json.loads(sorted((run / 'work').glob('attempt*/metrics.json'))[-1].read_text())
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
        worst = []
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
            worst.append((xy, deg))
        if worst:
            out['max_xy_m'], out['max_yaw_deg'] = float(max(w[0] for w in worst)), float(max(w[1] for w in worst))
    print(json.dumps(out))


if __name__ == '__main__':
    main()
