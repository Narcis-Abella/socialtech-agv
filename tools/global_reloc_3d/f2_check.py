"""One row per Phase-2 run: what the node did, what the gate let through, against the expectation written before the run.
  f2_check.py RUN_DIR 'EXPECT_REGEX'      regex is matched against  ATTEMPT_STATE(reason)/SUPERVISION_STATE(reason)
Invariants checked on every run, whatever the expectation (monotonic times, one host):
  P1 a pose is published only by an accepted attempt          G1 no non-zero gated command before LOCALIZED
  G2 no non-zero gated command later than GATE_CLOSE_S after the revocation (LOCALIZATION_LOST, or the node being killed/stopped)
  T1 an attempt record exists (the node reached a terminal state)     O1 no orphan processes     C1 no wrong_accept / pose_without_acceptance"""
import glob
import json
import os
import re
import sys

GATE_CLOSE_S = .7     # permit TTL 0.5 s + gate tick 0.05 s + scheduling margin


def load(p):
    return json.load(open(p)) if os.path.exists(p) and os.path.getsize(p) else None


def check(d, expect='.*'):
    mf = sorted(glob.glob(d + '/work*/attempt*/metrics.json'))
    m = load(mf[-1]) if mf else None
    sup = load(os.path.dirname(mf[-1]) + '/supervision.json') if mf else None
    ev, probe, cost = load(d + '/evaluation.json'), load(d + '/gate_probe.json'), load(d + '/cost_summary.json')
    inj = [l.split(' ', 1) for l in open(d + '/injected.txt')] if os.path.exists(d + '/injected.txt') else []
    r = {'run': os.path.basename(d), 'expect': expect}
    if m is None:
        mi = sorted(glob.glob(d + '/work*/attempt*/metrics_interrupted.json'))
        r.update(observed='NO_TERMINAL' + (f"[{load(mi[-1])['state']}]" if mi else ''), verdict='FAIL', fails='T1')
        return r
    t0 = m.get('first_input_mono')
    obs = f"{m['state']}({m['reason']})" + (f"/{sup['state']}({sup['reason']})" if sup else '')
    fails = []
    if m['initialpose'] is not None and not m['accepted']:
        fails.append('P1')
    node_sig = [float(t) for t, c in inj if 'reloc_node' in c and 'CONT' not in c]
    lost = sup and sup.get('lost_mono')
    loc = sup and sup.get('localized_mono')
    revoke = min([x for x in [lost] + node_sig if x], default=None)
    nz = (probe or {}).get('nonzero_intervals', [])
    if probe is None:
        fails.append('G?')
    if any(a < (loc or float('inf')) for a, b in nz):
        fails.append('G1')
    late = max((b - revoke for a, b in nz), default=None) if revoke else None
    if late is not None and late > GATE_CLOSE_S:
        fails.append('G2')
    orph = open(d + '/orphans.txt').read().strip() if os.path.exists(d + '/orphans.txt') else '?'
    if orph != 'fastlio=0 search=0 register=0':
        fails.append('O1')
    cls = (ev or {}).get('class', '')
    if cls in ('wrong_accept', 'pose_without_acceptance'):
        fails.append('C1')
    if not re.fullmatch(expect, obs):
        fails.append('EXPECT')
    first_inj = min((float(t) for t, c in inj), default=None)
    r.update(observed=obs, verdict='FAIL' if fails else 'PASS', fails=' '.join(fails), **{
        'class': cls, 'accepted': m['accepted'], 'pose_published': m['initialpose'] is not None, 'accepted_stage': m.get('accepted_stage') or '',
        'attempt_end_s': round(m['total_s']['attempt_end_from_first_input'], 2),
        'accept_s': m['total_s']['protocol_accept_from_first_input'] and round(m['total_s']['protocol_accept_from_first_input'], 2),
        'localized_s': round(loc - t0, 2) if loc and t0 else '', 'lost_s': round(lost - t0, 2) if lost and t0 else '',
        'fault_s': round(first_inj - t0, 2) if first_inj and t0 else '',
        'fault_to_lost_s': round(lost - first_inj, 2) if lost and first_inj else '',
        'gate_nonzero': sum(1 for _ in nz) and (probe or {}).get('out_nonzero'), 'gate_open_s': round(sum(b - a for a, b in nz), 2),
        'gate_last_nonzero_after_revoke_s': round(late, 3) if late is not None else '',
        'permit_true': (sup or {}).get('permit', {}).get('true', 0), 'monitor_max_xy_m': sup and sup['monitor'] and round(sup['monitor']['max_xy_m'], 3),
        'monitor_max_tf_gap_s': sup and sup['monitor'] and round(sup['monitor']['max_tf_gap_s'], 2),
        'odom_max_speed': m.get('odom_max_speed_mps') and round(max(m['odom_max_speed_mps'], (sup or {}).get('odom_max_speed_mps') or 0), 2),
        'xy_err_m': (ev or {}).get('initialpose_error', {}).get('xy_m', ''), 'pss_peak_mb': cost and round(cost['total_excl_bag_play']['pss_peak_mb'], 1),
        'orphans': orph})
    return r


if __name__ == '__main__':
    r = check(sys.argv[1].rstrip('/'), sys.argv[2] if len(sys.argv) > 2 else '.*')
    if len(sys.argv) > 3:
        import csv
        new = not os.path.exists(sys.argv[3])
        with open(sys.argv[3], 'a', newline='') as f:
            w = csv.DictWriter(f, list(COLS := ['run', 'expect', 'observed', 'verdict', 'fails', 'class', 'accepted', 'pose_published', 'accepted_stage', 'attempt_end_s', 'accept_s',
                                                'localized_s', 'lost_s', 'fault_s', 'fault_to_lost_s', 'gate_nonzero', 'gate_open_s', 'gate_last_nonzero_after_revoke_s', 'permit_true',
                                                'monitor_max_xy_m', 'monitor_max_tf_gap_s', 'odom_max_speed', 'xy_err_m', 'pss_peak_mb', 'orphans']), restval='')
            if new:
                w.writeheader()
            w.writerow(r)
    print(r['run'], r['verdict'], r.get('fails', ''), '|', r['observed'], '|', ' '.join(f'{k}={r[k]}' for k in ('class', 'localized_s', 'fault_s', 'lost_s', 'fault_to_lost_s', 'gate_open_s', 'gate_last_nonzero_after_revoke_s', 'orphans') if r.get(k) not in ('', None)))
