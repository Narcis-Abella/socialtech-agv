"""One row per Phase-2 run: what the node did, what the gate let through, against the expectation written before the run.
  f2_check.py RUN_DIR 'EXPECT_REGEX' [TABLE.csv [EVIDENCE_EXPR [CONTROL]]]
The regex is matched against ATTEMPT_STATE(reason)/SUPERVISION_STATE(reason) as it stood when the harness began to stop the run
(teardown_mono.txt): what the stack does while it is being torn down is reported apart (`after_teardown`) and never scored.
EVIDENCE_EXPR is a Python expression that must be true for the run to count at all: proof that the requested anomaly really
happened and that FAST-LIO2 really received data (variables: relay[imu|lidar][received|forwarded|dropped|shifted|duplicated|swapped|
altered|max_matched_subscribers], odom, cloud = messages FAST-LIO2 produced, imu_nonmono, lidar_nonmono, imu_gap, lidar_gap, imu_n, lidar_n = messages the node received before the attempt ended, time_jump_logs, inj =
injections fired, inj_rc = their exit codes, gate = reasons the gate reported). False -> verdict INVALID, neither PASS nor FAIL.
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
IO_STALL_S = 3.       # the cost sampler period is 0.5 s


def load(p):
    return json.load(open(p)) if os.path.exists(p) and os.path.getsize(p) else None


def check(d, expect='.*', evidence='', control=''):
    mf = sorted(glob.glob(d + '/work*/attempt*/metrics.json'))
    m = load(mf[-1]) if mf else None
    sup = load(os.path.dirname(mf[-1]) + '/supervision.json') if mf else None
    ev, probe, cost = load(d + '/evaluation.json'), load(d + '/gate_probe.json'), load(d + '/cost_summary.json')
    inj = [l.split(' ', 1) for l in open(d + '/injected.txt')] if os.path.exists(d + '/injected.txt') else []
    r = {'run': os.path.basename(d), 'expect': expect, 'control': control, 'evidence': evidence}
    if m is None:
        mi = sorted(glob.glob(d + '/work*/attempt*/metrics_interrupted.json'))
        r.update(observed='NO_TERMINAL' + (f"[{load(mi[-1])['state']}]" if mi else ''), verdict='FAIL', fails='T1')
        return r
    t0 = m.get('first_input_mono')
    td = float(open(d + '/teardown_mono.txt').read()) if os.path.exists(d + '/teardown_mono.txt') else float('inf')
    after = ''
    if sup and sup.get('lost_mono') and sup['lost_mono'] > td:      # revoked only while the harness was stopping the run
        after = f"{sup['state']}({sup['reason']})"
        sup = {**sup, 'state': 'LOCALIZED', 'reason': None, 'lost_mono': None}
    obs = f"{m['state']}({m['reason']})" + (f"/{sup['state']}({sup['reason']})" if sup else '')
    fails = []
    rs = load(d + '/relay_stats.json') or {}
    st = m['streams']
    env = {'relay': {k.split('/')[-1]: v for k, v in rs.items()}, 'odom': st['odom']['received'], 'cloud': st['cloud']['received'],
           'imu_nonmono': st['imu']['nonmonotonic'], 'lidar_nonmono': st['lidar']['nonmonotonic'], 'imu_gap': st['imu']['max_gap_s'],
           'lidar_gap': st['lidar']['max_gap_s'], 'imu_n': st['imu']['received'], 'lidar_n': st['lidar']['received'],
           'time_jump_logs': sum('jump' in l.lower() for l in open(d + '/launch.log', errors='replace')) if os.path.exists(d + '/launch.log') else 0,
           'inj': len(inj), 'gate': [e[2] for e in (probe or {}).get('gate_status_edges', [])],
           'inj_rc': [int(l.split()[0][3:]) for l in open(d + '/injected_rc.txt')] if os.path.exists(d + '/injected_rc.txt') else []}
    # The NX's NVMe stalls every file write for up to 30 s a few times per hour (kernel: "nvme0: I/O ... timeout, completion polled").
    # The 2 Hz cost sampler writes a row per process and stops with it: a hole in its time axis is that stall, seen from inside the run.
    ts = sorted({float(l.split(',')[0]) for l in list(open(d + '/cost.csv'))[1:]}) if os.path.exists(d + '/cost.csv') else []
    stall = max((b - a for a, b in zip(ts, ts[1:])), default=0.)
    r['io_stall_s'] = round(stall, 1)
    try:
        valid = bool(eval(evidence, dict(env))) if evidence else True
        if stall > IO_STALL_S:
            valid = False; after += f' platform_io_stall:{stall:.1f}s'
    except Exception as e:
        valid = False; after += f' evidence_error:{type(e).__name__}:{e}'
    r.update(control=control, evidence=evidence, valid=valid, after_teardown=after.strip(),
             fault_evidence=json.dumps({'relay': {k: {x: y for x, y in v.items() if y} for k, v in env['relay'].items()}, 'lio_odom': env['odom'], 'lio_cloud': env['cloud'],
                                        'imu_nonmono': env['imu_nonmono'], 'lidar_nonmono': env['lidar_nonmono'], 'imu_gap_s': round(env['imu_gap'], 3),
                                        'lidar_gap_s': round(env['lidar_gap'], 3), 'inj': env['inj'], 'inj_rc': env['inj_rc'], 'imu_n': env['imu_n'], 'lidar_n': env['lidar_n'], 'time_jump_logs': env['time_jump_logs']}))
    if m['initialpose'] is not None and not m['accepted']:
        fails.append('P1')
    node_sig = [float(t) for t, c in inj if 'reloc_node' in c and 'CONT' not in c]
    node_sig = [x for x in node_sig if x < td]
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
    if cls == 'unscorable_reference_gap' and m['initialpose'] is not None:     # a published pose nobody could score is not a pass (h27)
        fails.append('C?')
    if not re.fullmatch(expect, obs):
        fails.append('EXPECT')
    first_inj = min((float(t) for t, c in inj), default=None)
    r.update(observed=obs, verdict='INVALID' if not valid else 'FAIL' if fails else 'PASS', fails=' '.join(fails), **{
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
    r = check(sys.argv[1].rstrip('/'), sys.argv[2] if len(sys.argv) > 2 else '.*', *sys.argv[4:6])
    if len(sys.argv) > 3:
        import csv
        new = not os.path.exists(sys.argv[3])
        with open(sys.argv[3], 'a', newline='') as f:
            w = csv.DictWriter(f, list(COLS := ['run', 'expect', 'observed', 'verdict', 'fails', 'valid', 'control', 'evidence', 'fault_evidence', 'after_teardown', 'class', 'accepted', 'pose_published', 'accepted_stage', 'attempt_end_s', 'accept_s',
                                                'localized_s', 'lost_s', 'fault_s', 'fault_to_lost_s', 'gate_nonzero', 'gate_open_s', 'gate_last_nonzero_after_revoke_s', 'permit_true',
                                                'monitor_max_xy_m', 'monitor_max_tf_gap_s', 'odom_max_speed', 'xy_err_m', 'pss_peak_mb', 'orphans', 'io_stall_s']), restval='')
            if new:
                w.writeheader()
            w.writerow(r)
    print(r['run'], r['verdict'], r.get('fails', ''), '|', r['observed'], ('(after teardown: ' + r['after_teardown'] + ')') if r.get('after_teardown') else '', '|', ' '.join(f'{k}={r[k]}' for k in ('class', 'localized_s', 'fault_s', 'lost_s', 'fault_to_lost_s', 'gate_open_s', 'gate_last_nonzero_after_revoke_s', 'orphans') if r.get(k) not in ('', None)))
