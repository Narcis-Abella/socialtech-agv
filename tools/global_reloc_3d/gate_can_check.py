"""Verdict of gate_can_test.sh from the CAN frames the real tracer_base wrote (ID 0x111: int16 big-endian linear mm/s, angular mrad/s)."""
import json
import re
import sys

GATE_CLOSE_S = .7
out = sys.argv[1]
frames = []
for line in open(out + '/candump.log'):
    m = re.match(r'\s*\((\d+\.\d+)\)\s+\S+\s+111\s+\[\d\]\s+((?:[0-9A-F]{2}\s*)+)', line)
    if m:
        b = bytes.fromhex(m.group(2).replace(' ', ''))
        frames.append((float(m.group(1)), int.from_bytes(b[0:2], 'big', signed=True), int.from_bytes(b[2:4], 'big', signed=True)))
phases = json.load(open(out + '/permit_phases.json'))
bypass = float(open(out + '/bypass_epoch.txt').read())
rows, ok = [], True
for p, nxt in zip(phases, phases[1:] + [{'start_epoch': bypass}]):
    a, b = p['start_epoch'], nxt['start_epoch']
    f = [x for x in frames if a <= x[0] < b]
    nz = [x for x in f if x[1] or x[2]]
    row = {'phase': p['value'], 'seconds': round(b - a, 2), 'frames': len(f), 'nonzero': len(nz),
           'first_nonzero_after_s': round(nz[0][0] - a, 3) if nz else None, 'last_nonzero_after_s': round(nz[-1][0] - a, 3) if nz else None}
    if p['value'] == 'true':
        row['pass'] = len(nz) > 0 and nz[0][0] - a >= .15          # opens, and not before the third heartbeat
    else:                                                          # false / silent / exit: the wire goes to zero within the bound
        row['pass'] = all(x[0] - a <= GATE_CLOSE_S for x in nz) and (len(f) > len(nz) if rows and rows[-1]['phase'] == 'true' else True)
    ok &= row['pass']; rows.append(row)
kill_t, end_t = float(open(out + '/gate_kill_epoch.txt').read()), float(open(out + '/end_epoch.txt').read())
by = [x for x in frames if bypass <= x[0] < bypass + 2.]
before = [x for x in frames if x[0] < kill_t]
after = [x for x in frames if kill_t <= x[0] <= end_t]
ids = {}
for line in open(out + '/candump_all.log'):
    m = re.match(r'\s*\(\d+\.\d+\)\s+\S+\s+([0-9A-F]+)\s', line)
    if m:
        ids[m.group(1)] = ids.get(m.group(1), 0) + 1
res = {'frames_total': len(frames), 'phases': rows, 'gate_enforced': bool(ok and frames),
       'bypass_direct_publish_nonzero_frames': sum(1 for x in by if x[1] or x[2]),
       'can_ids_seen': ids,
       'gate_killed_while_open': {'last_frame_before_kill': before[-1][1:] if before else None, 'frames_after_kill': len(after),
                                  'nonzero_frames_after_kill': sum(1 for x in after if x[1] or x[2]),
                                  'last_frame_after_kill_at_s': round(after[-1][0] - kill_t, 3) if after else None,
                                  'meaning': 'no frame after the kill and a non-zero last frame = the last command stands until the base firmware times out'},
       'note': 'bypass > 0 is expected: a publisher on the gated topic itself is not stopped by the gate, only reported by it'}
print(json.dumps(res, indent=1))
