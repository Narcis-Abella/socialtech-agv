"""Time profile of the angular speed at each stage inside a window: turn_profile.py SAMPLES START_S END_S [BIN_S=0.5]   (START/END relative to the first sample, as turn_segments.py prints).
Per bin: mean w of cmd_vel_nav (controller), cmd_vel_smoothed, cmd_vel (after collision_monitor) and base_odom (what the base reports), rad/s, signed; '.' = no sample."""
import sys

from turn_segments import STAGES, load

by = load(sys.argv[1])
start, end = float(sys.argv[2]), float(sys.argv[3])
size = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5
t0 = min(t for s in by.values() for t, _ in s)
print(f"{'t(s)':>6} " + " ".join(f"{n[:10]:>10}" for n in STAGES))
t = start
while t < end:
    cells = []
    for n in STAGES:
        w = [v for ts, v in by[n] if t <= ts - t0 < t + size]
        cells.append(f"{sum(w) / len(w):10.3f}" if w else f"{'.':>10}")
    print(f"{t:6.1f} " + " ".join(cells))
    t += size
