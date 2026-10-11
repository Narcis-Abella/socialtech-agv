"""Sensitivity study: copy a frozen method input with seeded point dropout on fit/check.

Changes only how many query points are kept (a proxy for scan-pattern and timing
variation); the map and frames are untouched. Truth-free.
"""
import argparse
import json
import shutil
from pathlib import Path

import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input', required=True)
    p.add_argument('--out', required=True)
    p.add_argument('--drop', type=float, default=.2)
    p.add_argument('--seed', type=int, required=True)
    a = p.parse_args()
    src, out = Path(a.input), Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    rng = np.random.default_rng(a.seed)
    for name in ('fit.xyz', 'check.xyz'):
        pts = np.loadtxt(src / name)
        np.savetxt(out / name, pts[rng.random(len(pts)) >= a.drop], fmt='%.6f')
    shutil.copyfile(src / 'map.xyz', out / 'map.xyz')
    m = json.loads((src / 'manifest.json').read_text())
    m['perturbation'] = {'source': str(src), 'drop': a.drop, 'seed': a.seed}
    (out / 'manifest.json').write_text(json.dumps(m, indent=2) + '\n')


if __name__ == '__main__':
    main()
