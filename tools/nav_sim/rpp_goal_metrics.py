"""Hunting at the goal: scores the trials of one or more run dirs (trials.py series) on what happens after the robot first gets within the xy tolerance of the goal.
usage: rpp_goal_metrics.py <label>=<run dir> [<label>=<run dir> ...] | rpp_goal_metrics.py --selftest
Per trial, from the first row with the ESTIMATED distance to the goal < XY_TOL (0.20 m, see below) to the end of the series:
  dwell    seconds from that instant to the end (the real robot spent 42 s here in the goal of 889 s of the 29 Oct bag)
  away     share of the in-place rotation cycles (|nav_vx| < 0.01, |nav_wz| > 0.03) whose sign is opposite to the yaw error to the goal (est yaw)
  flips    sign changes of nav_wz among consecutive in-place rotation cycles
  turned   total |change of the true yaw| (rad) after that instant
Also whether the goal was reached (status SUCCEEDED) and the true distance / yaw error at the end. Distributions over all trials of a label: median, p95, max, N."""
import glob
import json
import math
import os
import sys

import numpy as np

XY_TOL = 0.20   # the goal checker uses 0.15 on the pose of the controller; the recorded "est" pose differs from it by 1-2 cm, so the entry radius is a bit wider
COLS = ["t", "gt_x", "gt_y", "gt_yaw", "est_x", "est_y", "est_yaw", "nav_vx", "nav_wz"]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def trial_metrics(rec, xy_tol=XY_TOL):
    c = rec["cols"]; R = np.array(rec["rows"], dtype=float); gx, gy, gyaw = rec["goal"]; ix = {k: c.index(k) for k in COLS}
    t = R[:, ix["t"]]; d_est = np.hypot(R[:, ix["est_x"]] - gx, R[:, ix["est_y"]] - gy); d_gt = np.hypot(R[:, ix["gt_x"]] - gx, R[:, ix["gt_y"]] - gy)
    out = {"id": rec["id"], "status": rec.get("status"), "reached": str(rec.get("status")).upper() in ("SUCCEEDED", "4"), "duration": rec.get("duration"), "d_end_gt": float(d_gt[-1]),
           "yaw_end_gt": abs(wrap(gyaw - R[-1, ix["gt_yaw"]])), "dwell": float("nan"), "away": float("nan"), "flips": 0, "turned": 0.0, "rot_cycles": 0}
    inside = np.where(d_est < xy_tol)[0]
    if len(inside) == 0: return out
    k = inside[0]; out["dwell"] = float(t[-1] - t[k]); S = R[k:]
    rot = (np.abs(S[:, ix["nav_vx"]]) < 0.01) & (np.abs(S[:, ix["nav_wz"]]) > 0.03); out["rot_cycles"] = int(rot.sum())
    if rot.any():
        err = np.array([wrap(gyaw - y) for y in S[:, ix["est_yaw"]]]); w = S[:, ix["nav_wz"]]
        out["away"] = float(np.mean(np.sign(w[rot]) != np.sign(err[rot]))); sg = np.sign(w[rot]); out["flips"] = int((sg[1:] != sg[:-1]).sum())
    out["turned"] = float(np.abs(np.diff(np.unwrap(S[:, ix["gt_yaw"]]))).sum())
    return out


def summarize(label, rows):
    def q(vals):
        v = np.array([x for x in vals if not (isinstance(x, float) and math.isnan(x))], dtype=float)
        return "      n/a" if len(v) == 0 else f"{np.median(v):7.2f} {np.percentile(v, 95):7.2f} {v.max():7.2f} (N={len(v)})"
    n = len(rows); print(f"== {label}: {n} trials, reached {sum(r['reached'] for r in rows)}/{n}, entered the {XY_TOL * 100:.0f} cm zone in {sum(not math.isnan(r['dwell']) for r in rows)}")
    print(f"   {'':16s}{'median':>7} {'p95':>7} {'max':>7}")
    for name, key in (("dwell s", "dwell"), ("away share", "away"), ("sign flips", "flips"), ("turned rad", "turned"), ("trial duration s", "duration"), ("final true dist m", "d_end_gt"), ("final true yaw err", "yaw_end_gt")):
        print(f"   {name:16s}{q([r[key] for r in rows])}")


def selftest():
    t = np.arange(0, 20, 0.02); n = len(t); gx = gy = 0.0
    def rec(wfun, d0=0.5):
        d = np.where(t < 2, d0 - (d0 - 0.1) * t / 2, 0.1); rows = []; yaw = 0.0
        for i in range(n):
            w = wfun(t[i]) if t[i] >= 2 else 0.0; yaw += w * 0.02; rows.append([t[i], d[i], 0, yaw, d[i], 0, yaw, 0.0, w])
        return {"id": "x", "goal": [0, 0, 1.0], "cols": COLS, "rows": rows, "status": "SUCCEEDED", "duration": 20.0}
    good = trial_metrics(rec(lambda tt: 0.4 if tt < 4.5 else 0.0)); assert good["away"] < 0.05 and good["flips"] == 0 and abs(good["dwell"] - 18.48) < 0.1, good   # the estimated distance falls 0.5 -> 0.1 m in 2 s: it crosses XY_TOL = 0.20 m at 1.5 s, 19.98 - 1.5 s of dwell
    bad = trial_metrics(rec(lambda tt: 0.4 if int(tt * 2) % 2 == 0 else -0.1)); assert bad["flips"] > 10 and 0.2 < bad["away"] < 0.6, bad
    far = trial_metrics(rec(lambda tt: 0.4, d0=0.5) | {"status": "FAILED"}); assert not far["reached"]
    print("selftest ok")


def main(args):
    for a in args:
        label, path = a.split("=", 1); rows = []
        for f in sorted(glob.glob(os.path.join(path, "trial_*.json"))): rows.append(trial_metrics(json.load(open(f))))
        summarize(label, rows) if rows else print(f"== {label}: no trial_*.json in {path}")


if __name__ == "__main__":
    selftest() if sys.argv[1] == "--selftest" else main(sys.argv[1:])
