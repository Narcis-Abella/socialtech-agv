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


def test_convergence_time_is_when_the_estimate_error_drops_and_stays_below_the_threshold():
    tr = straight_trial(est_offset=0.0)
    for row in tr["rows"]:
        row[COLS.index("est_x")] = row[COLS.index("gt_x")] + max(0.0, 1.0 - 0.1 * row[0])      # error 1.0 m decaying 0.1 m/s: below 0.3 m at t = 7 s
    r = a.trial_metrics(tr, ctx())
    assert abs(r["converge_s"] - 7.0) < 0.1
    never = straight_trial(est_offset=0.8)
    assert a.trial_metrics(never, ctx())["converge_s"] is None
    good = straight_trial(est_offset=0.05)
    assert a.trial_metrics(good, ctx())["converge_s"] == 0.0                                      # converged from the start
    dip = straight_trial(est_offset=0.0)
    for row in dip["rows"]:
        row[COLS.index("est_x")] = row[COLS.index("gt_x")] + (0.05 if row[0] < 3.0 else 0.0) + (0.6 if 3.0 <= row[0] < 4.0 else 0.0)   # good, then a 1 s excursion, then good again
    assert abs(a.trial_metrics(dip, ctx())["converge_s"] - 4.0) < 0.1                             # the excursion resets the clock
    nan_start = straight_trial(est_offset=0.0)
    for row in nan_start["rows"]:
        if row[0] < 2.0:
            row[COLS.index("est_x")] = float("nan")                                               # no transform yet is not "converged"
    assert abs(a.trial_metrics(nan_start, ctx())["converge_s"] - 2.0) < 0.1
    nan_always = straight_trial(est_offset=0.0)
    for row in nan_always["rows"]:
        row[COLS.index("est_x")] = float("nan")
    assert a.trial_metrics(nan_always, ctx())["converge_s"] is None


def test_idle_fraction_ignores_the_ground_truth_update_rate():
    tr = straight_trial()
    for i, row in enumerate(tr["rows"]):
        if i % 5:                                          # the ground truth only updates every 5th sample (like /gt/pose at 20 Hz against 50 Hz rows)
            row[1:3] = tr["rows"][i - i % 5][1:3]
    assert a.trial_metrics(tr, ctx())["idle_frac"] < 0.02  # it is driving the whole time
    stop = straight_trial()
    n = len(stop["rows"])
    for row in stop["rows"][n // 2:]:
        row[1:3] = stop["rows"][n // 2][1:3]                # stands still for the second half
    assert abs(a.trial_metrics(stop, ctx())["idle_frac"] - 0.5) < 0.05


def test_aggregate_uses_distributions():
    rows = [{"id": f"p{i}", "tag": "r0", "success": i % 4 != 0, "reached": i % 4 != 0, "duration": 10.0 + i, "spl": 0.8} for i in range(8)]
    g = a.aggregate(rows)
    assert g["n"] == 8 and g["successes"] == 6 and g["success_lo"] < 0.75 < g["success_hi"]
    assert g["duration"]["median"] == 14.0 and g["duration"]["n"] == 6          # only reached trials count for times


def test_clearance_mid_ignores_the_first_and_last_metre_where_the_start_and_goal_poses_are_given():
    occ = np.zeros((60, 200), bool)
    occ[28:33, 16:25] = True                                       # a block beside the start pose only (about 0.1 m from the footprint at the start, x 0.8-1.25)
    r = a.trial_metrics(straight_trial(), ctx(occ))
    assert r["clearance_min"] < 0.2 and r["clearance_mid"] > 0.3


def test_stops_reached_counts_the_via_poses_the_robot_came_within_half_a_metre_of():
    tr = straight_trial()
    tr["via"] = [[3.0, 1.0, 0.0], [3.0, 4.0, 0.0]]                 # the first is on the path, the second 3 m off it
    assert a.trial_metrics(tr, ctx())["stops_reached"] == 0.5
    assert "stops_reached" not in a.trial_metrics(straight_trial(), ctx())


def test_an_aborted_trial_with_almost_no_rows_still_has_every_summary_key():
    tr = straight_trial(status="aborted")
    tr["rows"] = tr["rows"][:2]                                    # Nav2 refused the goal at once (e.g. START_OCCUPIED): nothing was recorded
    r = a.trial_metrics(tr, ctx())
    assert not r["reached"] and r["spl"] == 0.0 and r["collisions"] == 0
    assert r["converge_s"] is None and r["stuck_start"] is False
    full = a.trial_metrics(straight_trial(), ctx())
    assert {"converge_s", "stuck_start"} <= set(r) and {"converge_s", "stuck_start"} <= set(full)
    rows = [a.trial_metrics(tr, ctx()), full]
    assert a.aggregate(rows)["n"] == 2 and a.failure_table(rows) == {"aborted": 1}


def test_a_robot_that_never_moved_is_flagged_as_stuck_at_the_start():
    tr = straight_trial(status="aborted")
    for row in tr["rows"]:
        row[1:3] = tr["rows"][0][1:3]                      # the ground truth never changes while Nav2 keeps commanding
    r = a.trial_metrics(tr, ctx())
    assert r["stuck_start"] and not r["reached"]
    assert not a.trial_metrics(straight_trial(), ctx())["stuck_start"]
    short = straight_trial(length=0.6, speed=0.3)           # a real but very short trial is not "stuck"
    assert not a.trial_metrics(short, ctx())["stuck_start"]
    tiny = straight_trial(length=0.2, speed=0.1)            # moved little in 2 s: a goal right next to the start, not a robot that cannot move
    assert not a.trial_metrics(tiny, ctx())["stuck_start"]


def test_exclude_drops_pair_ids_from_every_repetition():
    rows = [{"id": f"p{i:02d}", "tag": f"r{k}"} for k in range(2) for i in range(4)]
    out = a.exclude(rows, ["p01", "p03"])
    assert sorted({r["id"] for r in out}) == ["p00", "p02"] and len(out) == 4
    assert a.exclude(rows, []) == rows and a.exclude(rows, None) == rows


def test_error_names_decode_the_nav2_codes():
    assert a.error_name(0) == "none" and a.error_name(104) == "PATIENCE_EXCEEDED" and a.error_name(103) == "INVALID_PATH"
    assert a.error_name(203) == "START_OUTSIDE_MAP" and a.error_name(205) == "START_OCCUPIED" and a.error_name(208) == "NO_VALID_PATH"
    assert a.error_name(4242) == "code 4242" and a.error_name(-1) == "code -1"


def test_failure_table_counts_by_cause_and_status():
    rows = [{"reached": True, "status": "succeeded", "error_code": 0}, {"reached": False, "status": "aborted", "error_code": 104}, {"reached": False, "status": "aborted", "error_code": 104},
            {"reached": False, "status": "timeout", "error_code": -1}, {"reached": False, "status": "succeeded", "error_code": 0, "final_err_xy": 0.6}]
    t = a.failure_table(rows)
    assert t == {"PATIENCE_EXCEEDED": 2, "timeout": 1, "succeeded but away from the goal": 1}


def test_select_keeps_the_first_n_pair_ids_of_every_repetition():
    rows = [{"id": f"p{i:02d}", "tag": f"r{k}"} for k in range(2) for i in range(5)]
    sel = a.select(rows, 3)
    assert sorted({r["id"] for r in sel}) == ["p00", "p01", "p02"] and len(sel) == 6
    assert a.select(rows, None) == rows and len(a.select(rows, 99)) == 10


def test_compare_skips_pairs_without_a_convergence_time():
    base = [{"id": f"p{i}", "tag": "r0", "reached": True, "converge_s": 10.0 + i} for i in range(5)]
    cand = [{**r, "converge_s": r["converge_s"] - 4.0} for r in base]
    cand[1]["converge_s"] = None                                                                    # never converged in one pair
    d = a.compare(base, cand, ["converge_s"], seed=0)["converge_s"]
    assert d["n_pairs"] == 4 and abs(d["diff"] + 4.0) < 1e-9


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
