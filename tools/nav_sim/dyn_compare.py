"""Paired comparison of two sets of moving-obstacle runs (dyn_metrics.load_run per seed), paired by (seed, trial id).
usage: dyn_compare.py <base run dirs, comma-separated> <candidate run dirs, comma-separated>   (same number of dirs, same order of seeds)
reached / contact: (pairs only the base, pairs only the candidate) succeeded / avoided the contact, exact McNemar p. gap_body, duration, wait: paired mean difference candidate - base with a bootstrap 95 % CI;
duration only over pairs both reached."""
import sys

import numpy as np

import dyn_metrics
import stats


def _pairs(base, cand):
    out = []
    for s, (b, c) in enumerate(zip(base, cand)):
        bi, ci = {r["id"]: r for r in b}, {r["id"]: r for r in c}
        if set(bi) != set(ci):
            raise ValueError(f"seed {s}: trial ids differ: {sorted(set(bi) ^ set(ci))}")
        out += [(bi[i], ci[i]) for i in bi]
    return out


def compare(base, cand):
    allp = _pairs(base, cand)
    pairs = [(b, c) for b, c in allp if b["started"] and c["started"]]   # a trial where either obstacle never started is not an encounter
    res = {"n": len(pairs), "not_started": len(allp) - len(pairs)}
    res["reached"] = stats.discordant([b["reached"] for b, _ in pairs], [c["reached"] for _, c in pairs])
    res["contact"] = stats.discordant([not b["contact"] for b, _ in pairs], [not c["contact"] for _, c in pairs])
    for k, p in (("reached", res["reached"]), ("contact", res["contact"])):
        res[k + "_p"] = stats.mcnemar_exact(*p)
    for key in ("gap_body", "wait_s"):
        xs = [(b[key], c[key]) for b, c in pairs if b[key] is not None and c[key] is not None]
        res[key] = stats.paired_bootstrap_ci(np.array([x for x, _ in xs]), np.array([y for _, y in xs]))
    both = [(b["duration"], c["duration"]) for b, c in pairs if b["reached"] and c["reached"]]
    res["n_duration"] = len(both)
    res["duration"] = stats.paired_bootstrap_ci(np.array([x for x, _ in both]), np.array([y for _, y in both])) if both else (float("nan"),) * 3
    res["totals"] = {"reached": (sum(b["reached"] for b, _ in pairs), sum(c["reached"] for _, c in pairs)), "contact": (sum(b["contact"] for b, _ in pairs), sum(c["contact"] for _, c in pairs))}
    return res


def main(argv):
    base = [dyn_metrics.load_run(d + "/out") if not d.endswith("/out") else dyn_metrics.load_run(d) for d in argv[0].split(",")]
    cand = [dyn_metrics.load_run(d + "/out") if not d.endswith("/out") else dyn_metrics.load_run(d) for d in argv[1].split(",")]
    r = compare(base, cand)
    f = lambda t: f"{t[0]:+.2f} [{t[1]:+.2f}, {t[2]:+.2f}]"
    print(f"pairs {r['n']} (left out, obstacle not released: {r['not_started']})  reached base {r['totals']['reached'][0]} -> cand {r['totals']['reached'][1]}  only-base/only-cand {r['reached']}  McNemar p={r['reached_p']:.3f}")
    print(f"contacts base {r['totals']['contact'][0]} -> cand {r['totals']['contact'][1]}  avoided only-base/only-cand {r['contact']}  McNemar p={r['contact_p']:.3f}")
    print(f"gap_body (m) {f(r['gap_body'])}   wait (s) {f(r['wait_s'])}   duration (s, {r['n_duration']} pairs both reached) {f(r['duration'])}")


if __name__ == "__main__":
    main(sys.argv[1:])
