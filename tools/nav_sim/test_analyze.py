"""Run: python3 -B test_analyze.py   (offline: numpy and scipy)"""
import math

import numpy as np

import analyze as a
import metrics as m

COLS = ["t", "gt_x", "gt_y", "gt_yaw", "est_x", "est_y", "est_yaw", "nav_vx", "nav_wz", "sm_vx", "sm_wz", "cmd_vx", "cmd_wz"]


def straight_trial(speed=0.3, length=6.0, y=1.0, est_offset=0.1, status="succeeded", end_short=0.0):
    """Robot driving along +x at constant speed from x=1 for `length` metres; the estimate is est_offset ahead of the truth."""
    t = np.arange(0, length / speed, 0.02)
    x = 1.0 + speed * t
    rows = [[float(tt), float(xx), y, 0.0, float(xx) + est_offset, y, 0.0, speed, 0.0, speed, 0.0, speed, 0.0] for tt, xx in zip(t, x)]
    return {"id": "p00", "tag": "r0", "status": status, "error_code": 0, "duration": float(t[-1]), "recoveries": 1, "shortest": length, "init_err": 0.05,
            "goal": [1.0 + length - end_short, y, 0.0], "start": [1.0, y, 0.0], "cols": COLS, "rows": rows,
            "plan_first": [[1.0, y], [1.0 + length, y]], "plan_last": [[1.0, y], [1.0 + length, y]]}


def ctx(occ=None):
    occ = np.zeros((60, 200), bool) if occ is None else occ
    return {"dist": m.distance_to_occupied(occ, 0.05), "origin": (0.0, 0.0), "res": 0.05, "footprint": (0.70, 0.58), "tol": 0.25}


def test_clean_straight_run():
    r = a.trial_metrics(straight_trial(), ctx())
    assert r["success"] and r["reached"]
    assert abs(r["path_len"] - 6.0) < 0.05 and abs(r["spl"] - 1.0) < 0.02
    assert r["final_err_xy"] < 0.05 and r["recoveries"] == 1
    assert r["jerk_lin_rms"] < 1e-6 and r["vx_reversals"] == 0
    assert abs(r["loc_err_median"] - 0.1) < 1e-9 and r["xtrack_p95"] < 1e-9
    assert r["collisions"] == 0 and r["clearance_min"] > 0.3        # open map: far from every wall (the map edge counts as unknown, not as an obstacle)


def test_reported_success_without_reaching_the_goal_is_not_reached():
    r = a.trial_metrics(straight_trial(end_short=1.0), ctx())
    assert r["success"] and not r["reached"] and r["final_err_xy"] > 0.9
    assert r["spl"] == 0.0                                  # SPL only counts trials that really reached the goal


def test_failure_has_zero_spl():
    r = a.trial_metrics(straight_trial(status="aborted"), ctx())
    assert not r["success"] and r["spl"] == 0.0


def test_collision_is_counted_with_the_footprint_not_the_centre():
    occ = np.zeros((60, 200), bool)
    occ[:, 100] = True                                   # wall at x = 5.0 .. 5.05, in the robot's lane
    r = a.trial_metrics(straight_trial(length=5.5), ctx(occ))
    assert r["collisions"] > 0 and r["clearance_min"] <= 0.0
    # centre-only measure would still be >0 at the first colliding sample: the footprint front edge is 0.35 m ahead


def test_oscillating_command_is_counted():
    tr = straight_trial()
    for i, row in enumerate(tr["rows"]):
        row[COLS.index("cmd_vx")] = 0.3 if (i // 25) % 2 == 0 else -0.3
    r = a.trial_metrics(tr, ctx())
    assert r["vx_reversals"] > 5 and r["jerk_lin_rms"] > 1.0


def test_aggregate_uses_distributions():
    rows = [{"id": f"p{i}", "tag": "r0", "success": i % 4 != 0, "reached": i % 4 != 0, "duration": 10.0 + i, "spl": 0.8} for i in range(8)]
    g = a.aggregate(rows)
    assert g["n"] == 8 and g["successes"] == 6 and g["success_lo"] < 0.75 < g["success_hi"]
    assert g["duration"]["median"] == 14.0 and g["duration"]["n"] == 6          # only reached trials count for times


def test_compare_pairs_by_id_and_tag():
    base = [{"id": f"p{i}", "tag": "r0", "success": True, "reached": True, "duration": 10.0 + i, "spl": 0.8, "jerk_lin_rms": 1.0} for i in range(6)]
    cand = [{**r, "duration": r["duration"] - 2.0, "jerk_lin_rms": 0.5} for r in base]
    cand[0]["success"] = cand[0]["reached"] = False
    out = a.compare(base, cand, ["duration", "jerk_lin_rms"], seed=0)
    d = out["duration"]
    assert d["n_pairs"] == 5 and abs(d["diff"] + 2.0) < 1e-9 and d["hi"] < 0          # failed pair is dropped for the time metric
    assert out["_success"]["only_base"] == 1 and out["_success"]["only_cand"] == 0


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
