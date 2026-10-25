"""Metrics, summary table and paired comparison of recorded trials (offline: numpy, scipy).
usage: analyze.py summarize <run dir> [--map map.yaml] [--csv out.csv] [--first N] [--exclude p19,p20]
       analyze.py compare <base run dir> <candidate run dir> [--map map.yaml] [--first N] [--exclude ids]
<run dir> holds trial_*.json (trials.py). `reached` = Nav2 says succeeded AND the ground-truth pose is within --tol (0.25 m) of the goal. Clearance and collisions use the footprint against the
occupied cells of the map (unknown cells are not obstacles). Times, SPL and smoothness are compared only on pairs both runs reached."""
import glob
import json
import math
import os
import sys

import numpy as np

import metrics as m
import scenarios as sc
import stats

DEFAULT_MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "maps", "loyola_gt.yaml")      # registered to the simulated world
FOOTPRINT = (0.70, 0.58)
HZ = 50.0                              # sampling rate of trials.py rows
COMPARED = ["duration", "path_len", "spl", "final_err_xy", "recoveries", "jerk_lin_rms", "jerk_lin_p95", "acc_lin_p95", "jerk_ang_rms", "acc_ang_p95", "vx_reversals",
            "xtrack_p95", "loc_err_p95", "converge_s", "clearance_min", "clearance_mid", "idle_frac"]


def load_trials(run_dir):
    recs = [json.load(open(p)) for p in sorted(glob.glob(os.path.join(run_dir, "**", "trial_*.json"), recursive=True))]
    if not recs:
        raise SystemExit(f"no trial_*.json under {run_dir}")
    return recs


def context(map_yaml=DEFAULT_MAP, tol=0.25, footprint=FOOTPRINT):
    mp = sc.load_map(map_yaml)
    return {"dist": m.distance_to_occupied(mp["occ"], mp["res"]), "origin": mp["origin"], "res": mp["res"], "footprint": footprint, "tol": tol}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def trial_metrics(rec, ctx):
    col = {c: i for i, c in enumerate(rec["cols"])}
    rows = np.asarray(rec["rows"], float)
    out = {"id": rec["id"], "tag": rec["tag"], "status": rec["status"], "error_code": rec["error_code"], "duration": rec["duration"], "recoveries": rec["recoveries"],
           "init_err": rec["init_err"], "success": rec["status"] == "succeeded"}
    if len(rows) < 3:
        return {**out, "reached": False, "spl": 0.0, "collisions": 0, "converge_s": None, "stuck_start": False}
    t, gt = rows[:, col["t"]], rows[:, [col["gt_x"], col["gt_y"]]]
    est = rows[:, [col["est_x"], col["est_y"]]]
    out["path_len"] = m.path_length(gt)
    out["stuck_start"] = bool(out["path_len"] < 0.3 and rec["duration"] > 10.0)   # a pose from which the robot cannot move (physical contact the lidar does not see): invalid pair
    out["final_err_xy"] = float(math.hypot(rows[-1, col["gt_x"]] - rec["goal"][0], rows[-1, col["gt_y"]] - rec["goal"][1]))
    out["final_err_yaw"] = abs(wrap(rows[-1, col["gt_yaw"]] - rec["goal"][2]))
    out["reached"] = bool(out["success"] and out["final_err_xy"] <= ctx["tol"])
    out["spl"] = m.spl(out["reached"], rec.get("shortest") or 0.0, out["path_len"])
    for key, v in (("lin", rows[:, col["cmd_vx"]]), ("ang", rows[:, col["cmd_wz"]])):
        _, vv = m.resample(t, v, 0.05)
        acc, jerk = m.acc_jerk(np.arange(len(vv)) * 0.05, vv)
        out[f"acc_{key}_p95"], out[f"jerk_{key}_rms"], out[f"jerk_{key}_p95"] = m.p95(np.abs(acc)), m.rms(jerk), m.p95(np.abs(jerk))
    out["vx_mean"] = float(np.mean(rows[:, col["cmd_vx"]]))
    out["vx_reversals"] = m.sign_reversals(rows[:, col["cmd_vx"]], 0.01)
    out["wz_reversals"] = m.sign_reversals(rows[:, col["cmd_wz"]], 0.02)
    k = max(1, int(round(0.5 * HZ)))      # speed over 0.5 s windows: /gt/pose updates at ~20 Hz, so per-row differences are zero most of the time
    speed = np.hypot(*(gt[k:] - gt[:-k]).T) / np.maximum(t[k:] - t[:-k], 1e-6)
    out["idle_frac"] = float(np.mean(speed < 0.02)) if len(speed) else 0.0
    plan = np.asarray(rec["plan_first"], float)
    if len(plan) > 1:
        xt = m.cross_track(plan, gt[::5])
        out["xtrack_rms"], out["xtrack_p95"], out["xtrack_max"] = m.rms(xt), m.p95(xt), float(xt.max())
    full = np.hypot(*(est - gt).T)
    bad = ~(full < 0.3)                       # NaN (no transform yet) counts as not converged
    out["converge_s"] = None if bad[-1] else (0.0 if not bad.any() else float(t[np.flatnonzero(bad)[-1] + 1] - t[0]))   # seconds until the estimate is within 0.3 m of the truth for good
    err = full[np.isfinite(full)]
    out["loc_err_median"] = float(np.median(err)) if err.size else math.nan
    out["loc_err_p95"], out["loc_err_max"] = (m.p95(err), float(err.max())) if err.size else (math.nan, math.nan)
    poses = rows[::5][:, [col["gt_x"], col["gt_y"], col["gt_yaw"]]]
    cl = np.array([m.footprint_clearance(ctx["dist"], ctx["origin"], ctx["res"], x, y, yaw, *ctx["footprint"]) for x, y, yaw in poses])
    cl = np.minimum(cl, 5.0)        # beyond 5 m it is "far": keeps inf out of the statistics
    out["clearance_min"], out["clearance_p5"] = float(cl.min()), float(np.percentile(cl, 5))
    away = (np.hypot(*(poses[:, :2] - rec["start"][:2]).T) > 1.0) & (np.hypot(*(poses[:, :2] - rec["goal"][:2]).T) > 1.0)
    out["clearance_mid"] = float(cl[away].min()) if away.any() else out["clearance_min"]     # start and goal poses are given by the scenario, not chosen by the planner
    out["collisions"] = int(np.sum(cl <= 0.0))
    return out


def aggregate(rows):
    n = len(rows)
    k = sum(r["reached"] for r in rows)
    lo, hi = stats.wilson(k, n)
    ok = [r for r in rows if r["reached"]]
    g = {"n": n, "successes": k, "success_lo": lo, "success_hi": hi}
    for key in ("duration", "spl", "path_len", "final_err_xy", "jerk_lin_rms", "jerk_ang_rms", "acc_lin_p95", "xtrack_p95", "loc_err_p95", "clearance_min", "clearance_mid", "vx_reversals", "recoveries", "idle_frac"):
        g[key] = stats.summary([r[key] for r in ok if key in r and r[key] == r[key]])
    g["collision_trials"] = sum(r.get("collisions", 0) > 0 for r in rows)
    return g


def compare(base, cand, metric_names, seed=0):
    """Pairs by (id, tag). Continuous metrics on pairs both reached; success on all pairs (McNemar exact)."""
    bi = {(r["id"], r["tag"]): r for r in base}
    pairs = [(bi[(r["id"], r["tag"])], r) for r in cand if (r["id"], r["tag"]) in bi]
    only_b, only_c = stats.discordant([b["reached"] for b, _ in pairs], [c["reached"] for _, c in pairs])
    out = {"_success": {"pairs": len(pairs), "only_base": only_b, "only_cand": only_c, "p": stats.mcnemar_exact(only_b, only_c)}}
    both = [(b, c) for b, c in pairs if b["reached"] and c["reached"]]
    for name in metric_names:
        usable = [(b[name], c[name]) for b, c in both if b.get(name) is not None and c.get(name) is not None and b[name] == b[name] and c[name] == c[name]]
        if len(usable) < 2:
            out[name] = {"n_pairs": len(usable)}
            continue
        a, b = np.array(usable).T
        est, lo, hi = stats.paired_bootstrap_ci(a, b, seed=seed)
        out[name] = {"n_pairs": len(usable), "base": float(np.mean(a)), "cand": float(np.mean(b)), "diff": est, "lo": lo, "hi": hi}
    return out


def fmt(x):
    return "--" if x is None or (isinstance(x, float) and x != x) else f"{x:.3g}"


def print_summary(rows, name=""):
    g = aggregate(rows)
    print(f"== {name}  n={g['n']}  reached={g['successes']} ({g['successes'] / g['n']:.0%}, Wilson95 [{g['success_lo']:.0%}, {g['success_hi']:.0%}])  trials with a collision={g['collision_trials']}")
    print(f"{'metric (reached trials)':28s} {'n':>3s} {'median':>9s} {'p95':>9s} {'max':>9s}")
    for k, v in g.items():
        if isinstance(v, dict) and v.get("n"):
            print(f"{k:28s} {v['n']:3d} {fmt(v['median']):>9s} {fmt(v['p95']):>9s} {fmt(v['max']):>9s}")
    stuck = [r["id"] for r in rows if r.get("stuck_start")]
    if stuck:
        print("  stuck at the start (physical contact, probably an invalid pair; exclude with --exclude):", ",".join(stuck))
    bad = [r for r in rows if not r["reached"]]
    if bad:
        print("  failures by cause:", ", ".join(f"{k} x{v}" for k, v in sorted(failure_table(rows).items(), key=lambda kv: -kv[1])))
    for r in bad:
        print(f"  not reached: {r['tag']} {r['id']} status={r['status']} {error_name(r['error_code'])} final_err={fmt(r.get('final_err_xy'))} recoveries={r['recoveries']} t={fmt(r['duration'])}s")


ERRORS = {0: "none", 100: "UNKNOWN", 101: "INVALID_CONTROLLER", 102: "TF_ERROR", 103: "INVALID_PATH", 104: "PATIENCE_EXCEEDED", 105: "FAILED_TO_MAKE_PROGRESS", 106: "NO_VALID_CONTROL",
          107: "CONTROLLER_TIMED_OUT", 200: "UNKNOWN", 201: "INVALID_PLANNER", 202: "TF_ERROR", 203: "START_OUTSIDE_MAP", 204: "GOAL_OUTSIDE_MAP", 205: "START_OCCUPIED", 206: "GOAL_OCCUPIED",
          207: "TIMEOUT", 208: "NO_VALID_PATH"}      # nav2_msgs FollowPath (1xx) and ComputePathToPose (2xx) in Jazzy 1.3.13


def error_name(code):
    return ERRORS.get(code, f"code {code}")


def failure_table(rows):
    """Count of trials not reached, by the cause Nav2 reports (or timeout / succeeded but away from the goal)."""
    out = {}
    for r in rows:
        if r["reached"]:
            continue
        key = "succeeded but away from the goal" if r["status"] == "succeeded" else (error_name(r["error_code"]) if r["error_code"] > 0 else r["status"])
        out[key] = out.get(key, 0) + 1
    return out


def exclude(rows, ids):
    """Rows without the given pair ids (every repetition)."""
    return rows if not ids else [r for r in rows if r["id"] not in set(ids)]


def select(rows, first):
    """Rows of the first `first` pair ids (sorted), every repetition; all rows when first is None."""
    if first is None:
        return rows
    keep = set(sorted({r["id"] for r in rows})[:first])
    return [r for r in rows if r["id"] in keep]


def parse(argv):
    opts = {"--map": DEFAULT_MAP, "--csv": None, "--tol": "0.25", "--first": None, "--exclude": None}
    pos = []
    i = 0
    while i < len(argv):
        if argv[i] in opts:
            opts[argv[i]] = argv[i + 1]
            i += 2
        else:
            pos.append(argv[i])
            i += 1
    return pos, opts


def main(argv):
    pos, opts = parse(argv)
    ctx = context(opts["--map"], float(opts["--tol"]))
    first = int(opts["--first"]) if opts["--first"] else None
    excl = opts["--exclude"].split(",") if opts["--exclude"] else None
    if pos[0] == "summarize":
        rows = select(exclude([trial_metrics(r, ctx) for r in load_trials(pos[1])], excl), first)
        print_summary(rows, os.path.basename(os.path.normpath(pos[1])))
        if opts["--csv"]:
            keys = sorted({k for r in rows for k in r})
            open(opts["--csv"], "w").write(",".join(keys) + "\n" + "\n".join(",".join(str(r.get(k, "")) for k in keys) for r in rows) + "\n")
    elif pos[0] == "compare":
        b, c = (select(exclude([trial_metrics(r, ctx) for r in load_trials(p)], excl), first) for p in pos[1:3])
        res = compare(b, c, COMPARED)
        s = res["_success"]
        print(f"pairs={s['pairs']}  reached only by base={s['only_base']}  only by candidate={s['only_cand']}  McNemar exact p={s['p']:.3g}")
        print(f"{'metric':16s} {'pairs':>5s} {'base':>9s} {'cand':>9s} {'diff':>9s} {'CI95 lo':>9s} {'CI95 hi':>9s}  (paired bootstrap on pairs both reached)")
        for k in COMPARED:
            v = res[k]
            if v.get("diff") is None:
                print(f"{k:16s} {v['n_pairs']:5d}")
                continue
            sig = "*" if (v["lo"] > 0 or v["hi"] < 0) else " "
            print(f"{k:16s} {v['n_pairs']:5d} {fmt(v['base']):>9s} {fmt(v['cand']):>9s} {fmt(v['diff']):>9s} {fmt(v['lo']):>9s} {fmt(v['hi']):>9s} {sig}")
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main(sys.argv[1:])
