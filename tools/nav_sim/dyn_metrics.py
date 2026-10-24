"""Safety and outcome of trials with moving obstacles (trials.py JSON with "dyn" / "dyn_log"), from the ground truth of both.
usage: dyn_metrics.py <run dir> [<run dir> ...]    prints one line per trial and the totals.
gap_body = closest approach (m) between the robot's physical body (BODY, from the simulated model) and the obstacle; gap_footprint the same against Nav2's footprint (FOOTPRINT, the planning/collision-monitor shape).
Both assume the robot's ground-truth pose is the centre of the rectangle. contact = gap_body == 0. reached = Nav2 succeeded and the ground-truth pose ends within GOAL_TOL of the goal."""
import glob
import json
import math
import os
import sys

import dyn

BODY, FOOTPRINT, GOAL_TOL, NEAR = (0.60, 0.40), (0.70, 0.58), 0.25, 3.0   # m


def trial_metrics(rec):
    col = {c: i for i, c in enumerate(rec["cols"])}
    rows = rec["rows"]
    size = {d["model"]: d["size"] for d in rec.get("dyn", [])}
    log = {}
    for t, model, x, y, yaw in rec.get("dyn_log", []):
        log.setdefault(model, []).append((t, x, y, yaw))
    gap_b = gap_f = None
    idx = {m: 0 for m in log}
    for r in rows:
        t, x, y, yaw = r[col["t"]], r[col["gt_x"]], r[col["gt_y"]], r[col["gt_yaw"]]
        for m, entries in log.items():
            while idx[m] + 1 < len(entries) and entries[idx[m] + 1][0] <= t:      # latest command sent at or before t
                idx[m] += 1
            if entries[idx[m]][0] > t:
                continue
            _, ox, oy, oyaw = entries[idx[m]]
            if math.hypot(ox - x, oy - y) > NEAR:
                continue
            o = (ox, oy, oyaw, size[m][0], size[m][1])
            b, f = dyn.rect_distance((x, y, yaw, *BODY), o), dyn.rect_distance((x, y, yaw, *FOOTPRINT), o)
            gap_b = b if gap_b is None else min(gap_b, b)
            gap_f = f if gap_f is None else min(gap_f, f)
    if gap_b is None and log:
        gap_b = gap_f = NEAR                                                         # never within NEAR: report the bound, not a false zero
    ts = [r[col["t"]] for r in rows]
    dt = sorted(b - a for a, b in zip(ts, ts[1:]))[len(ts) // 2] if len(ts) > 2 else 0.0
    moving = False
    wait = 0.0
    for r in rows:
        v = abs(r[col["cmd_vx"]])
        moving = moving or v > 0.02
        wait += dt if moving and v < 0.02 else 0.0
    last = rows[-1] if rows else None
    reached = bool(rec["status"] == "succeeded" and last and math.hypot(last[col["gt_x"]] - rec["goal"][0], last[col["gt_y"]] - rec["goal"][1]) <= GOAL_TOL)
    return {"id": rec["id"], "tag": rec.get("tag"), "status": rec["status"], "reached": reached, "duration": rec["duration"], "recoveries": rec.get("recoveries", 0),
            "gap_body": gap_b, "gap_footprint": gap_f, "contact": gap_b is not None and gap_b <= 0.0, "wait_s": wait}


def load_run(d):
    return [trial_metrics(json.load(open(f))) for f in sorted(glob.glob(os.path.join(d, "trial_*.json")))]


def main(dirs):
    for d in dirs:
        res = load_run(d)
        print(f"== {d}")
        for r in res:
            g = "   -  " if r["gap_body"] is None else f"{r['gap_body']:6.2f}"
            print(f"{r['tag']} {r['id']:10s} {r['status']:10s} reached={int(r['reached'])} t={r['duration']:6.1f}s gap_body={g} gap_fp={'-' if r['gap_footprint'] is None else round(r['gap_footprint'], 2)} wait={r['wait_s']:5.1f}s rec={r['recoveries']}{' CONTACT' if r['contact'] else ''}")
        n = len(res)
        print(f"N={n} reached={sum(r['reached'] for r in res)} contacts={sum(r['contact'] for r in res)} "
              f"gap_body min={min([r['gap_body'] for r in res if r['gap_body'] is not None], default=float('nan')):.2f}")


if __name__ == "__main__":
    main(sys.argv[1:])
