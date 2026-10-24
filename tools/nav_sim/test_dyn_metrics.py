"""Run: python3 -B test_dyn_metrics.py"""
import json
import os
import tempfile

import dyn_metrics as dm

COLS = ["t", "gt_x", "gt_y", "gt_yaw", "est_x", "est_y", "est_yaw", "nav_vx", "nav_wz", "sm_vx", "sm_wz", "cmd_vx", "cmd_wz", "odom_x", "odom_y", "odom_yaw"]


def trial(xs, obstacle, goal=(4.0, 0.0), status="succeeded", vx=0.5):
    """Robot driving along y = 0 through x values `xs` at 10 Hz; the obstacle stays at `obstacle` (x, y) from t = 0."""
    rows = [[0.1 * i, x, 0.0, 0.0, x, 0.0, 0.0, vx, 0, vx, 0, vx, 0, x, 0, 0] for i, x in enumerate(xs)]
    log = [[0.1 * i, "b_walker", obstacle[0], obstacle[1], 0.0] for i in range(len(xs))]
    return {"id": "t", "tag": "r0", "goal": [goal[0], goal[1], 0.0], "status": status, "duration": 0.1 * len(xs), "recoveries": 0, "cols": COLS, "rows": rows,
            "dyn": [{"model": "b_walker", "size": [0.5, 0.5, 1.7]}], "dyn_log": log}


def test_gap_is_the_closest_approach_between_body_and_obstacle_not_between_centres():
    r = dm.trial_metrics(trial([0.0, 0.5, 1.0, 1.5, 2.0], obstacle=(4.0, 0.0)))
    # robot body 0.6 long: front face at x + 0.3; obstacle back face at 4.0 - 0.25 = 3.75; closest robot x = 2.0 -> gap 3.75 - 2.3 = 1.45
    assert abs(r["gap_body"] - 1.45) < 1e-6
    assert abs(r["gap_footprint"] - (3.75 - 2.35)) < 1e-6                 # Nav2 footprint is 0.70 long: front face at x + 0.35


def test_a_robot_that_reaches_the_obstacle_has_zero_gap_and_is_flagged_as_contact():
    r = dm.trial_metrics(trial([0.0, 1.0, 2.0, 3.0, 3.6], obstacle=(4.0, 0.0)))
    assert r["gap_body"] == 0.0 and r["contact"]


def test_samples_before_the_obstacle_log_starts_do_not_count():
    t = trial([0.0, 1.0, 2.0, 3.0, 3.6], obstacle=(4.0, 0.0))
    t["dyn_log"] = [e for e in t["dyn_log"] if e[0] >= 0.35]              # the obstacle only appears after the robot sample at x = 3.0
    assert dm.trial_metrics(t)["gap_body"] == 0.0 and dm.trial_metrics(dict(t, dyn_log=[]))["gap_body"] is None


def test_reached_needs_success_and_ground_truth_within_goal_tolerance():
    ok = dm.trial_metrics(trial([0.0, 3.9, 4.05], obstacle=(0.0, 20.0)))
    assert ok["reached"]
    far = dm.trial_metrics(trial([0.0, 1.0, 2.0], obstacle=(0.0, 20.0)))      # Nav2 says succeeded, the robot is 2 m away
    assert not far["reached"]
    assert not dm.trial_metrics(trial([0.0, 3.9, 4.05], obstacle=(0.0, 20.0), status="aborted"))["reached"]


def test_wait_time_counts_samples_with_zero_commanded_speed_after_the_first_motion():
    xs = [0.0, 0.1, 0.2, 0.2, 0.2, 0.2, 0.3]
    t = trial(xs, obstacle=(0.0, 20.0))
    for i in (3, 4, 5):
        t["rows"][i][11] = 0.0
    assert abs(dm.trial_metrics(t)["wait_s"] - 0.3) < 1e-9


def test_gap_is_the_minimum_over_time_not_the_last_sample():
    r = dm.trial_metrics(trial([0.0, 1.0, 2.0, 3.0, 2.0], obstacle=(4.0, 0.0)))        # closest at x = 3.0: front face 3.3, obstacle face 3.75
    assert abs(r["gap_body"] - 0.45) < 1e-6


def test_obstacle_poses_are_not_used_before_the_first_command():
    t = trial([3.6, 3.0, 2.0], obstacle=(4.0, 0.0))
    t["dyn_log"] = [e for e in t["dyn_log"] if e[0] >= 0.15]                           # at t = 0 and 0.1 the obstacle is still parked far away
    assert abs(dm.trial_metrics(t)["gap_body"] - 1.45) < 1e-6


def test_time_standing_before_the_first_motion_is_not_waiting():
    t = trial([0.0, 0.0, 0.1, 0.2, 0.3], obstacle=(0.0, 20.0))
    t["rows"][0][11] = t["rows"][1][11] = 0.0
    assert dm.trial_metrics(t)["wait_s"] == 0.0


def test_an_obstacle_that_never_comes_within_three_metres_reports_that_bound():
    assert dm.trial_metrics(trial([0.0, 1.0, 2.0], obstacle=(0.0, 20.0)))["gap_body"] == dm.NEAR


def test_a_trial_whose_obstacle_never_started_is_flagged_as_not_an_encounter():
    t = trial([0.0, 1.0, 2.0], obstacle=(4.0, 0.0))
    t["dyn_log"], t["dyn_unstarted"] = [], ["b_walker"]
    r = dm.trial_metrics(t)
    assert r["started"] is False and r["gap_body"] is None
    assert dm.trial_metrics(trial([0.0, 1.0, 2.0], obstacle=(4.0, 0.0)))["started"] is True


def test_summarize_a_run_dir_reads_every_trial_file():
    with tempfile.TemporaryDirectory() as d:
        for k, xs in enumerate(([0.0, 3.9, 4.05], [0.0, 1.0, 2.0])):
            t = trial(xs, obstacle=(0.0, 20.0))
            t["id"] = f"x{k}"
            json.dump(t, open(os.path.join(d, f"trial_r0_x{k}.json"), "w"))
        res = dm.load_run(d)
    assert [r["id"] for r in res] == ["x0", "x1"] and [r["reached"] for r in res] == [True, False]


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
