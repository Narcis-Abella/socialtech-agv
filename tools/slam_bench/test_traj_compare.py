"""Checks traj_compare.py on synthetic trajectories. Run: python3 test_traj_compare.py"""
import numpy as np
from scipy.spatial.transform import Rotation

import traj_compare as tc

RNG = np.random.default_rng(5)


def path(n=500):
    """A winding 10 Hz trajectory as alidarState rows: t x y z qx qy qz qw."""
    t = np.arange(n) * 0.1
    xyz = np.c_[8 * np.sin(t / 7), 5 * np.sin(t / 3) + 0.3 * t, 0.2 * np.sin(t)]
    q = Rotation.from_euler("z", np.degrees(np.arctan2(np.gradient(xyz[:, 1]), np.gradient(xyz[:, 0])))[:, None], degrees=True).as_quat()
    return np.c_[t, xyz, q]


def moved(P, yaw=30.0, shift=(4.0, -2.0, 0.5)):
    """The same trajectory seen from another origin: what a different run's arbitrary start frame does."""
    R = Rotation.from_euler("z", yaw, degrees=True)
    Q = P.copy()
    Q[:, 1:4] = R.apply(P[:, 1:4]) + shift
    Q[:, 4:8] = (R * Rotation.from_quat(P[:, 4:8])).as_quat()
    return Q


def test_identical_trajectories_have_full_coverage_and_no_error():
    P = path()
    r = tc.compare(P, P)
    assert r["coverage"] == 1.0 and r["ate_rmse"] < 1e-9 and r["ate_max"] < 1e-9


def test_a_different_start_frame_is_aligned_away():
    P = path()
    r = tc.compare(P, moved(P))
    assert r["coverage"] == 1.0 and r["ate_rmse"] < 1e-6, r


def test_missing_scans_lower_the_coverage_not_the_error():
    P = path()
    keep = RNG.random(len(P)) > 0.2
    r = tc.compare(P, moved(P)[keep])
    assert abs(r["coverage"] - keep.mean()) < 1e-9 and r["ate_rmse"] < 1e-6, r


def test_noise_and_drift_show_up_in_the_error():
    P = path()
    noisy = moved(P); noisy[:, 1:4] += RNG.normal(0, 0.01, (len(P), 3))
    assert 0.005 < tc.compare(P, noisy)["ate_rmse"] < 0.03
    drift = moved(P); drift[:, 1] += np.linspace(0, 1.0, len(P))          # 1 m of drift along the run
    assert tc.compare(P, drift)["ate_rmse"] > 0.1


def test_poses_a_few_milliseconds_apart_still_match_but_distant_ones_do_not():
    P = path()
    near = moved(P); near[:, 0] += 0.004
    assert tc.compare(P, near)["coverage"] == 1.0
    far = moved(P); far[:, 0] += 0.05                                       # half a scan period: a different scan
    assert tc.compare(P, far)["coverage"] < 0.05


if __name__ == "__main__":
    test_identical_trajectories_have_full_coverage_and_no_error()
    test_a_different_start_frame_is_aligned_away()
    test_missing_scans_lower_the_coverage_not_the_error()
    test_noise_and_drift_show_up_in_the_error()
    test_poses_a_few_milliseconds_apart_still_match_but_distant_ones_do_not()
    print("ok")
