"""Summary and paired comparison of the paths asked from each global planner by plan_bench.py (offline: numpy, scipy).
usage: plan_analysis.py summarize <run dir> --scenario scenario.json [--map map.yaml]
       plan_analysis.py compare <run dir> <planner A (base)> <planner B> --scenario scenario.json [--map map.yaml]
<run dir>/plans.json comes from plan_bench.py. detour = path length / reference shortest length of the pair; planning time is the one the planner server reports (all requests, failures included)."""
import json
import math
import os
import sys

import numpy as np

import analyze
import plan_metrics as pm
import stats

PATH_KEYS = ("length", "clearance_min", "turning", "max_curvature", "sharp_turns")


def load(run_dir):
    return json.load(open(os.path.join(run_dir, "plans.json")))


def _metrics(row, shortest, ctx):
    m = pm.path_metrics(row["path"], ctx)
    m["detour"] = m["length"] / shortest[row["id"]] if shortest.get(row["id"]) else math.nan
    return m


def summarize(plans, shortest, ctx):
    out = {}
    for pid in plans["planners"]:
        rows = [r for r in plans["rows"] if r["planner"] == pid]
        ok = [r for r in rows if r["ok"]]
        mets = [_metrics(r, shortest, ctx) for r in ok]
        fails = {}
        for r in rows:
            if not r["ok"]:
                k = analyze.error_name(r["error_code"])
                fails[k] = fails.get(k, 0) + 1
        out[pid] = {"n": len(rows), "ok": len(ok), "failures": fails, "planning_time": stats.summary([r["planning_time"] for r in rows]),
                    "detour": stats.summary([m["detour"] for m in mets if m["detour"] == m["detour"]])}
        for k in PATH_KEYS:
            out[pid][k] = stats.summary([m[k] for m in mets])
    return out


def compare(plans, base, cand, names, shortest, ctx, seed=0):
    """Pairs by (id, rep). Path metrics on pairs both planned; planning time on all pairs; success by exact McNemar."""
    by = {p: {(r["id"], r["rep"]): r for r in plans["rows"] if r["planner"] == p} for p in (base, cand)}
    keys = sorted(set(by[base]) & set(by[cand]))
    ob, oc = stats.discordant([by[base][k]["ok"] for k in keys], [by[cand][k]["ok"] for k in keys])
    out = {"_success": {"pairs": len(keys), "only_base": ob, "only_cand": oc, "p": stats.mcnemar_exact(ob, oc)}}
    for name in names:
        vals = []
        for k in keys:
            b, c = by[base][k], by[cand][k]
            if name == "planning_time":
                vals.append((b["planning_time"], c["planning_time"]))
            elif b["ok"] and c["ok"]:
                vals.append((_metrics(b, shortest, ctx)[name], _metrics(c, shortest, ctx)[name]))
        vals = [v for v in vals if v[0] == v[0] and v[1] == v[1]]
        if len(vals) < 2:
            out[name] = {"n_pairs": len(vals)}
            continue
        a, b = np.array(vals).T
        est, lo, hi = stats.paired_bootstrap_ci(a, b, seed=seed)
        out[name] = {"n_pairs": len(vals), "base": float(a.mean()), "cand": float(b.mean()), "diff": est, "lo": lo, "hi": hi}
    return out


def _args(argv):
    opts, pos, i = {"--scenario": None, "--map": analyze.DEFAULT_MAP}, [], 0
    while i < len(argv):
        if argv[i] in opts:
            opts[argv[i]] = argv[i + 1]
            i += 2
        else:
            pos.append(argv[i])
            i += 1
    return pos, opts


def main(argv):
    pos, opts = _args(argv)
    if not opts["--scenario"] or not pos:
        raise SystemExit(__doc__)
    plans = load(pos[1])
    shortest = {t["id"]: t.get("shortest") for t in json.load(open(opts["--scenario"]))["trials"]}
    ctx = analyze.context(opts["--map"])
    if pos[0] == "summarize":
        s = summarize(plans, shortest, ctx)
        print(f"{'planner':22s} {'ok/n':>7s} {'time ms med':>11s} {'p95':>7s} {'max':>7s} {'detour med':>10s} {'clear min p5?':>13s} {'sharp max':>9s} {'curv max':>9s}")
        for pid, v in s.items():
            t, d = v["planning_time"], v["detour"]
            print(f"{pid:22s} {v['ok']:3d}/{v['n']:<3d} {1e3 * t['median']:11.1f} {1e3 * t['p95']:7.1f} {1e3 * t['max']:7.1f} {analyze.fmt(d.get('median')):>10s} "
                  f"{analyze.fmt(v['clearance_min'].get('min')):>13s} {analyze.fmt(v['sharp_turns'].get('max')):>9s} {analyze.fmt(v['max_curvature'].get('max')):>9s}  {v['failures'] or ''}")
    elif pos[0] == "compare":
        res = compare(plans, pos[2], pos[3], ["planning_time", "detour", "length", "clearance_min", "turning", "max_curvature", "sharp_turns"], shortest, ctx)
        s = res["_success"]
        print(f"pairs={s['pairs']} only {pos[2]}={s['only_base']} only {pos[3]}={s['only_cand']} McNemar p={s['p']:.3g}")
        for k, v in res.items():
            if k != "_success" and v.get("diff") is not None:
                print(f"{k:16s} n={v['n_pairs']:3d} base {analyze.fmt(v['base']):>8s} cand {analyze.fmt(v['cand']):>8s} diff {analyze.fmt(v['diff']):>8s} CI95 [{analyze.fmt(v['lo'])}, {analyze.fmt(v['hi'])}]")
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main(sys.argv[1:])
