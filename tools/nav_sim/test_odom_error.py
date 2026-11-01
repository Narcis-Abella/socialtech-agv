"""Run: python3 -B test_odom_error.py"""
import numpy as np

import odom_error as oe


def path(n=2000, step=0.01, turn=0.0005):
    """A gentle S-curve sampled every `step` m, 10 Hz-ish timestamps."""
    th = np.cumsum(np.full(n, turn) * np.sin(np.arange(n) / 300.0))
    xy = np.cumsum(step * np.c_[np.cos(th), np.sin(th)], axis=0)
    return np.arange(n) * 0.1, xy, th


def test_identical_paths_have_zero_error():
    t, xy, th = path()
    r = oe.window_errors(xy, th, xy, th, 1.0)
    assert len(r["frac"]) > 100 and np.max(r["frac"]) < 1e-9 and np.max(np.abs(r["yaw_per_m"])) < 1e-9


def test_scale_error_shows_up_as_along_track_fraction():
    t, xy, th = path()
    r = oe.window_errors(xy * 1.03, th, xy, th, 1.0)
    assert abs(np.median(r["along"]) - 0.03) < 5e-3 and abs(np.median(r["lateral"])) < 5e-3


def test_constant_yaw_drift_per_metre():
    t, xy, th = path()
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
    r = oe.window_errors(xy, th + 0.01 * s, xy, th, 2.0)
    assert abs(np.median(r["yaw_per_m"]) - 0.01) < 1e-3


def test_a_rigid_transform_of_the_estimate_changes_nothing():
    t, xy, th = path()
    c, s = np.cos(0.7), np.sin(0.7)
    R = np.array([[c, -s], [s, c]])
    r = oe.window_errors(xy @ R.T + [5, -3], th + 0.7, xy, th, 1.0)
    assert np.max(r["frac"]) < 1e-9


def test_windows_follow_path_length_not_time():
    t, xy, th = path()
    r = oe.window_errors(xy, th, xy, th, 1.0)
    assert abs(np.median(r["dist"]) - 1.0) < 0.02


def test_trials_mode_reads_odom_columns_and_skips_old_series(capsys=None):
    import io, json, os, tempfile, contextlib
    t, xy, th = path()
    cols = ["t", "gt_x", "gt_y", "gt_yaw", "odom_x", "odom_y", "odom_yaw"]
    rows = np.c_[t, xy, th, xy * 1.05, th].tolist()
    with tempfile.TemporaryDirectory() as d:
        json.dump({"cols": cols, "rows": rows}, open(os.path.join(d, "trial_r0_p00.json"), "w"))
        json.dump({"cols": cols[:4], "rows": [r[:4] for r in rows]}, open(os.path.join(d, "trial_r0_p01.json"), "w"))   # series without odometry
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            oe.main_trials(d, [1.0])
    line = buf.getvalue().strip().splitlines()[-1].split()
    assert abs(float(line[4]) - 0.05) < 5e-3, line          # along med = the 5 % scale error


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
    print("ok")
