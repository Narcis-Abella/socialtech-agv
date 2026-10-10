"""Verdict of gate_cell_test.sh from the CAN frames on the cell's virtual bus (ID 0x111: int16 big-endian linear mm/s, angular mrad/s)."""
import json
import os
import re
import sys

CLOSE_S = .7          # permit TTL 0.5 s + gate tick + margin, as f2_check.py
STOP_S = .5           # outlet / dead-man silence limit 0.3 s + one period + margin
out = sys.argv[1]
frames, ids = [], {}
for line in open(out + '/candump_all.log'):
    m = re.match(r'\s*\((\d+\.\d+)\)\s+\S+\s+([0-9A-F]+)\s+\[\d\]\s*((?:[0-9A-F]{2}\s*)*)', line)
    if not m:
        continue
    ids[m.group(2)] = ids.get(m.group(2), 0) + 1
    if m.group(2) == '111':
        b = bytes.fromhex(m.group(3).replace(' ', ''))
        frames.append((float(m.group(1)), int.from_bytes(b[0:2], 'big', signed=True), int.from_bytes(b[2:4], 'big', signed=True)))
ev = {}
for line in open(out + '/events.txt'):
    k, t = line.split(); ev[k] = float(t)


def window(a, b):
    f = [x for x in frames if a <= x[0] < b]
    nz = [x for x in f if x[1] or x[2]]
    return {'frames': len(f), 'nonzero': len(nz), 'first_nonzero_after_s': round(nz[0][0] - a, 3) if nz else None,
            'last_nonzero_after_s': round(nz[-1][0] - a, 3) if nz else None, 'zero_after_last_nonzero': sum(1 for x in f if not (x[1] or x[2]) and (not nz or x[0] > nz[-1][0]))}


phases = json.load(open(out + '/permit_phases.json'))
rows, ok = [], True
for p, nxt in zip(phases, phases[1:] + [{'start_epoch': ev['bypass_start']}]):
    r = {'phase': p['value'], **window(p['start_epoch'], nxt['start_epoch'])}
    if p['value'] == 'true':
        r['pass'] = r['nonzero'] > 0 and r['first_nonzero_after_s'] >= .15
    else:
        r['pass'] = r['nonzero'] == 0 or r['last_nonzero_after_s'] <= CLOSE_S
    ok &= r['pass']; rows.append(r)
res = {'frames_total': len(frames), 'can_ids_seen': ids, 'phases': rows, 'gate_enforced': bool(ok and frames)}
b = window(ev['bypass_start'], ev['bypass_end'])
res['bypass_closed_gate'] = {**b, 'socket_from_graph': open(out + '/bypass_socket.log').read().strip()[:120],
                             'graph_sees_socket_dir': open(out + '/graph_sees_sock.txt').read().strip()[:120],
                             'main_graph_sees_driver': 'tracer' in open(out + '/main_graph.txt').read(), 'pass': b['nonzero'] == 0}


def stop_after(name, until):
    w = window(ev[name], until)
    w['pass'] = (w['nonzero'] == 0 or w['last_nonzero_after_s'] <= STOP_S) and w['zero_after_last_nonzero'] > 0
    return w


res['source_dies_while_open'] = stop_after('source_stopped', ev['source_resumed'])
res['outlet_killed_while_open'] = stop_after('outlet_killed', ev['outlet_restarted'])
final = 'gate_killed' if 'gate_killed' in ev else 'driver_killed'
res['resumes_after_outlet_restart'] = window(ev['outlet_restarted'], ev[final])['nonzero'] > 0
res[final + '_while_open'] = stop_after(final, ev['end_final'])
for f in ('deadman.json', 'outlet_stats.json', 'outlet_stats2.json'):
    if os.path.exists(f'{out}/{f}'):
        res[f[:-5]] = json.load(open(f'{out}/{f}'))
res['all_pass'] = bool(res['gate_enforced'] and res['bypass_closed_gate']['pass'] and not res['bypass_closed_gate']['main_graph_sees_driver']
                       and all(res[k]['pass'] for k in ('source_dies_while_open', 'outlet_killed_while_open', final + '_while_open'))
                       and res['resumes_after_outlet_restart'])
print(json.dumps(res, indent=1))
