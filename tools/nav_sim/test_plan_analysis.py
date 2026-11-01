"""Run: python3 -B test_plan_analysis.py   (offline: numpy and scipy)"""
import json
import math
import os
import tempfile

import numpy as np

import metrics as m
import plan_analysis as pa

RES = 0.05


def ctx():
    return {"dist": m.distance_to_occupied(np.zeros((300, 400), bool), RES), "origin": (0.0, 0.0), "res": RES, "footprint": (0.70, 0.58)}


def line(a, b, n=40):
    return [list(q) for q in zip(np.linspace(a[0], b[0], n), np.linspace(a[1], b[1], n))]


def plans(rows):
    return {"planners": sorted({r["planner"] for r in rows}), "rows": rows}


def row(pid, planner, path, t=0.05, ok=True, code=0, rep=0):
    return {"id": pid, "planner": planner, "rep": rep, "ok": ok, "error_code": code, "planning_time": t, "wall_time": t + 0.01, "path": path}


SHORT = {"p00": 10.0, "p01": 10.0, "p02": 10.0}


def test_per_planner_summary_uses_distributions():
    rows = []
    for i, pid in enumerate(("p00", "p01", "p02")):
        rows.append(row(pid, "A", line((1, 2), (11, 2)), t=0.01 * (i + 1)))
        rows.append(row(pid, "B", line((1, 2), (6, 2)) + line((6, 2), (6, 7))[1:], t=0.10))               # a detour: 10 m... plus a corner
    rows.append(row("p02", "B", [], ok=False, code=208, t=0.2, rep=1))
    out = pa.summarize(plans(rows), SHORT, ctx())
    a, b = out["A"], out["B"]
    assert a["n"] == 3 and a["ok"] == 3 and abs(a["planning_time"]["median"] - 0.02) < 1e-9
    assert abs(a["detour"]["median"] - 1.0) < 1e-9 and abs(b["detour"]["median"] - 1.0) < 1e-9                  # both 10 m long here
    assert b["n"] == 4 and b["ok"] == 3 and b["failures"] == {"NO_VALID_PATH": 1}
    assert b["sharp_turns"]["max"] == 1 and a["sharp_turns"]["max"] == 0


def test_detour_ratio_is_length_over_reference():
    rows = [row("p00", "A", line((1, 2), (11, 2))), row("p01", "A", line((1, 2), (16, 2)))]
    out = pa.summarize(plans(rows), {"p00": 10.0, "p01": 10.0}, ctx())["A"]
    assert abs(out["detour"]["min"] - 1.0) < 1e-6 and abs(out["detour"]["max"] - 1.5) < 1e-6


def test_compare_pairs_planners_on_the_same_pair_ids():
    rows = []
    for pid in ("p00", "p01", "p02", "p03"):
        rows.append(row(pid, "A", line((1, 2), (11, 2)), t=0.10))
        rows.append(row(pid, "B", line((1, 2), (11, 2)), t=0.05))
    rows.append(row("p04", "A", line((1, 2), (11, 2)), t=0.10))                                                 # only A planned p04: not a pair
    out = pa.compare(plans(rows), "A", "B", ["planning_time"], SHORT | {"p03": 10.0, "p04": 10.0}, ctx())
    d = out["planning_time"]
    assert d["n_pairs"] == 4 and abs(d["diff"] + 0.05) < 1e-12 and d["hi"] < 0


def test_load_reads_plans_json():
    with tempfile.TemporaryDirectory() as d:
        json.dump(plans([row("p00", "A", line((1, 2), (11, 2)))]), open(os.path.join(d, "plans.json"), "w"))
        assert pa.load(d)["rows"][0]["id"] == "p00"


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
