"""Seeded scenario list for the software-restart campaign (Phase 2). Every start differs: window, start jitter, stationary lead-in,
bring-up lead, DDS domain. They are NOT independent trials: a few dozen windows of 3 bags in 2 buildings, on development data.
  gen_restart_cases.py SEED N WINDOWS_DIR [WINDOWS_DIR...] [--extra ds:window,ds:window,...] > cases.csv
window files: <ds>-cert.csv, <ds>-s2.csv, <ds>-stat.csv. Without --extra every window comes equally often. With it, every window
comes once and the listed ones (hard, rejecting or S3 windows, named before the campaign) fill the rest, shuffled in."""
import csv
import glob
import os
import random
import sys

DUR = {'eco01': 523., 'eco03': 271., 'hospital': 634.}     # bag lengths (AGENTS.md); a start needs 75 s of bag after it


def main():
    args = sys.argv[1:]
    extra = []
    if '--extra' in args:
        i = args.index('--extra'); extra = [tuple(x.split(':')) for x in args[i + 1].split(',')]; del args[i:i + 2]
    seed, n, dirs = int(args[0]), int(args[1]), args[2:]
    rng = random.Random(seed)
    windows = {}
    for d in dirs:
        for f in sorted(glob.glob(d + '/*.csv')):
            ds, kind = os.path.basename(f)[:-4].split('-', 1) if '-' in os.path.basename(f) else (None, None)
            if ds not in DUR or kind not in ('cert', 's2', 'stat'):
                continue
            for r in csv.DictReader(open(f)):
                off = float(r['capture_offset_s'])
                if DUR[ds] - off >= 80.:
                    windows.setdefault((ds, r['window']), off)
    keys = sorted(windows)
    order = []
    if extra:
        missing = [e for e in extra if e not in windows]
        if missing or len(keys) + len(extra) != n:
            sys.exit(f'--extra: unknown windows {missing} or {len(keys)} + {len(extra)} != {n}')
        order = keys + extra; rng.shuffle(order)
    while len(order) < n:                     # every window equally often, order shuffled
        block = keys[:]; rng.shuffle(block); order += block
    w = csv.writer(sys.stdout)
    w.writerow(['id', 'seed', 'ds', 'window', 'offset_s', 'jitter_s', 'pre_s', 'lead_s', 'domain'])
    for i, (ds, win) in enumerate(order[:n]):
        w.writerow([f'r{i:03d}', seed, ds, win, f'{windows[(ds, win)]:.6f}', f'{rng.uniform(-.5, .5):.3f}', f'{rng.uniform(2.5, 4.):.2f}',
                    rng.choice([10, 12, 14]), 100 + rng.randrange(100)])


if __name__ == '__main__':
    main()
