"""NX host: which runs overlap a storage stall? The board's NVMe logs `nvme0: I/O ... timeout, completion polled` a few times per
hour; the request that hit it was blocked for up to 30 s before that line. A run is flagged when such a line falls between its
start and 31 s after its end, and separately when its own cost sampler (2 Hz, one write per sample) shows a hole.
  io_stall_audit.py RUNS_DIR [RUNS_DIR ...] > stalls.csv        needs journalctl -k (no root)"""
import csv
import datetime
import glob
import json
import os
import re
import subprocess
import sys


def kernel_timeouts(since='2026-10-01'):
    out = subprocess.run(['journalctl', '-k', '--since', since, '--no-pager', '-o', 'short-unix'], capture_output=True, text=True).stdout
    return [float(l.split()[0]) for l in out.splitlines() if 'nvme' in l and 'timeout' in l]


def sampler_gap(d):
    p = d + '/cost.csv'
    if not os.path.exists(p):
        return ''
    t = sorted({float(l.split(',')[0]) for l in list(open(p))[1:] if l.strip()})
    return round(max((b - a for a, b in zip(t, t[1:])), default=0.), 1)


def main():
    ts = kernel_timeouts()
    w = csv.writer(sys.stdout)
    w.writerow(['run', 'start', 'duration_s', 'kernel_timeouts_at_s', 'sampler_max_gap_s', 'state'])
    n = 0
    for root in sys.argv[1:]:
        for d in sorted(glob.glob(root.rstrip('/') + '/*')):
            h = d + '/host.txt'
            m = re.search(r'start: (\S+)', open(h).read()) if os.path.exists(h) else None
            if not m:
                continue
            n += 1
            s, e = datetime.datetime.fromisoformat(m.group(1)).timestamp(), os.path.getmtime(h)
            hits = [round(t - s, 1) for t in ts if s <= t <= e + 31]
            gap = sampler_gap(d)
            if hits or (gap != '' and gap > 3):
                mf = sorted(glob.glob(d + '/work*/attempt*/metrics.json'))
                st = ''
                if mf:
                    mm = json.load(open(mf[-1])); st = f"{mm['state']}({mm['reason']})"
                w.writerow([os.path.basename(d), m.group(1), round(e - s, 1), ' '.join(map(str, hits)), gap, st])
    print(f'# {n} runs scanned, {len(ts)} kernel timeouts in the journal', file=sys.stderr)


if __name__ == '__main__':
    main()
