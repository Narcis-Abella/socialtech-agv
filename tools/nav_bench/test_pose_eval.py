"""Checks pose_eval.py on synthetic trajectories with known answers. Run: python3 test_pose_eval.py"""
import json
import os
import re
import tempfile

import numpy as np

import pose_eval as pe

RZ90 = np.array([[0, -1, 0, 5.0], [1, 0, 0, 0.0], [0, 0, 1, 0.5], [0, 0, 0, 1.0]])  # p_map = Rz(90) p + (5, 0, .5)
YAML = """image: map.pgm
mode: trinary
resolution: 0.25
origin: [-10.0, -10.0, 0.0]
negate: 0
# T_map_world (row-major; p_map = T p_ply):
# [0.0, -1.0, 0.0, 5.0]
# [1.0, 0.0, 0.0, 0.0]
# [0.0, 0.0, 1.0, 0.5]
# [0.0, 0.0, 0.0, 1.0]
"""


def yaw_quat(y):
    return np.array([0, 0, np.sin(y / 2), np.cos(y / 2)])


def write_tum(path, t, x, y, yaw, extra=None):
    rows = [[ti, xi, yi, 0, *yaw_quat(yw), *(extra or [])] for ti, xi, yi, yw in zip(t, x, y, yaw)]
    np.savetxt(path, rows, fmt="%.6f")


def test_planar_moves_poses_into_the_target_frame():
    out = pe.planar(np.array([[1.0, 0, 0]]), np.array([yaw_quat(0.0)]), RZ90)
    assert np.allclose(out, [[5.0, 1.0, np.pi / 2]]), out


def test_map_yaml_without_T_is_refused_and_with_T_is_parsed():
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "m.yaml")
        open(p, "w").write("image: m.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
        try:
            pe.read_map_yaml(p)
            raise AssertionError("accepted a YAML without T_map_world")
        except ValueError as e:
            assert "T_map_world" in str(e)
        open(p, "w").write(YAML)
        m = pe.read_map_yaml(p)
        assert m["resolution"] == 0.25 and m["origin"] == (-10.0, -10.0) and np.allclose(m["T_map_world"], RZ90)
        assert m["image"] == os.path.join(d, "map.pgm")


def test_pgm_rows_are_flipped_and_outside_is_255():
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "a.pgm")
        open(p, "wb").write(b"P5\n# comment\n3 2\n255\n" + bytes([0, 205, 254, 10, 20, 30]))  # top row first
        g = pe.read_pgm(p)
        assert g.shape == (2, 3) and g[0, 2] == 254
        v = pe.cells(g, 1.0, (0.0, 0.0), np.array([[0.5, 0.5], [2.5, 1.5], [9.0, 9.0]]))  # bottom-left, top-right, outside
        assert list(v) == [10, 254, 255], v


def test_evaluate_finds_convergence_loss_and_jumps():
    t = np.arange(0, 60, 0.5)
    ref = np.c_[0.5 * t, np.zeros_like(t), np.zeros_like(t)]
    err = np.exp(-t / 6) + ((t >= 30) & (t < 40))  # starts 1 m off, decays; 10 s of 1 m offset in the middle
    est = ref + np.c_[err, np.zeros_like(t), np.zeros_like(t)]
    r = pe.evaluate(t, est, ref.copy(), ref)  # odometry == reference: no drift
    assert r["converged"] and r["convergence_s"] == 7.5, r
    assert r["losses"] == 1 and r["map_odom_jumps"] == 2, r  # the offset steps in and out of map->odom
    assert abs(r["initial_pos_err_m"] - 1.0) < 1e-9 and abs(r["pos_err_m"]["max"] - 1.0) < 0.01, r


def test_jumps_while_converging_are_not_counted_as_map_odom_jumps():
    t = np.arange(0, 60, 0.5)
    ref = np.zeros((len(t), 3))
    early = np.where((t * 2).astype(int) % 2 == 0, 1.0, 0.5) * (t < 4)  # error flips 1.0 / 0.5 m every step until 4 s, then 0
    r = pe.evaluate(t, ref + np.c_[early, np.zeros_like(t), np.zeros_like(t)], ref.copy(), ref)
    assert r["converged"] and r["map_odom_jumps"] == 0 and r["map_odom_jumps_before_convergence"] == 8, r


def test_a_small_yaw_wobble_far_from_the_odom_origin_is_not_a_jump():
    t = np.arange(0, 60, 0.5)
    ref = np.c_[0.5 * t, np.zeros_like(t), np.zeros_like(t)]  # drives 30 m away from the odom origin
    wobble = np.radians(0.5) * np.where(np.arange(len(t)) % 2 == 0, 1.0, -1.0)  # position exact, yaw +-0.5 deg every update
    est = ref + np.c_[np.zeros_like(t), np.zeros_like(t), wobble]
    r = pe.evaluate(t, est, ref.copy(), ref)
    # map->odom itself moves ~26 cm per update at 30 m (lever arm of the yaw), but at the ROBOT the correction is 1 deg and ~1 cm
    assert r["map_odom_jumps"] == 0 and r["correction_at_robot_cm"]["p95"] < 5, r


def test_never_converging_run_is_reported_as_such():
    t = np.arange(0, 20, 0.5)
    ref = np.zeros((len(t), 3))
    est = ref + [2.0, 0, 0]
    r = pe.evaluate(t, est, ref.copy(), ref)
    assert not r["converged"] and r["convergence_s"] is None and r["losses"] == 0, r


def test_disjoint_clocks_fail_loudly():
    t = np.arange(0, 20, 0.5)
    try:
        pe.evaluate(t, np.full((len(t), 3), np.nan), np.zeros((len(t), 3)), np.zeros((len(t), 3)))
        raise AssertionError("accepted samples that do not overlap")
    except ValueError as e:
        assert "clocks" in str(e)


def test_level_rotation_puts_the_floor_normal_on_z_and_roundtrips_as_a_quaternion():
    with tempfile.TemporaryDirectory() as d:
        pitch = np.radians(1.0)  # the IMU frame at start is tilted 1 deg in the gravity-aligned SLAM world
        q0 = np.array([0, np.sin(pitch / 2), 0, np.cos(pitch / 2)])
        np.savetxt(f"{d}/ref.txt", [[0.0, 0, 0, 0, *q0], [0.1, 0, 0, 0, *q0]])
        q = pe.level_quat(f"{d}/ref.txt", RZ90)
        R_L = pe.quat_to_mat(q)
        assert np.allclose(R_L, RZ90[:3, :3] @ pe.quat_to_mat(q0)) and q[3] >= 0
        up_in_fastlio_frame = pe.quat_to_mat(q0).T @ [0, 0, 1]  # world up seen from the IMU frame at start
        assert np.allclose(R_L @ up_in_fastlio_frame, [0, 0, 1])  # RZ90 does not tilt, so up stays up
    for R in (np.eye(3), np.diag([1.0, -1, -1]), np.diag([-1.0, 1, -1]), np.diag([-1.0, -1, 1])):  # all branches (w = 0 included)
        assert np.allclose(pe.quat_to_mat(pe.mat_to_quat(R)), R), R


def test_initpose_message_carries_the_pose_and_an_explicit_covariance():
    msg = json.loads(pe.initpose_json((1.0, 2.0, np.pi / 2), 0.5, 15.0))
    assert msg["header"]["frame_id"] == "map" and msg["pose"]["pose"]["position"]["x"] == 1.0 and msg["pose"]["pose"]["position"]["y"] == 2.0
    q = msg["pose"]["pose"]["orientation"]
    assert abs(2 * np.arctan2(q["z"], q["w"]) - np.pi / 2) < 1e-6
    c = msg["pose"]["covariance"]
    assert len(c) == 36 and c[0] == c[7] == 0.25 and abs(c[35] - np.radians(15.0) ** 2) < 1e-9 and sum(1 for v in c if v) == 3, c


def test_initial_pose_is_pushed_along_the_heading_and_turned():
    x, y, yaw = pe.initial_pose(np.array([[1.0, 2.0, np.pi / 2]]), 1.0, 20.0)
    assert np.allclose([x, y], [1.0, 3.0]) and abs(yaw - np.radians(110)) < 1e-9


def scene(d, pixel):
    """A straight 30 s drive in the world frame, the AMCL poses of the same drive in the (rotated) map frame 5 cm off, and an all-`pixel` 160x160 map."""
    t = 1.7e9 + np.arange(0, 30, 0.1)
    wx, wy = 0.5 * (t - t[0]), np.zeros_like(t)  # world frame (the SLAM frame): straight line along x
    write_tum(f"{d}/ref.txt", t, wx, wy, np.zeros_like(t), extra=[9, 9, 9])  # alidarState has junk columns after the 8th
    write_tum(f"{d}/odom.tum", t, wx, wy, np.zeros_like(t))
    te = t[::5]
    write_tum(f"{d}/amcl.tum", te, np.full_like(te, 5.05), 0.5 * (te - t[0]), np.full_like(te, np.pi / 2), extra=[0.01, 0.01, 0.0])  # map frame
    open(f"{d}/map.yaml", "w").write(YAML)
    open(f"{d}/map.pgm", "wb").write(b"P5\n160 160\n255\n" + bytes([pixel]) * (160 * 160))
    return pe.report(f"{d}/amcl.tum", f"{d}/odom.tum", f"{d}/ref.txt", f"{d}/map.yaml")


def test_report_end_to_end_with_a_rotated_map_frame():
    with tempfile.TemporaryDirectory() as d:
        r = scene(d, 254)
        assert r["converged"] and abs(r["pos_err_m"]["median"] - 0.05) < 1e-6 and r["yaw_err_deg"]["max"] < 1e-6, r
        assert r["losses"] == 0 and r["map_odom_jumps"] == 0 and r["ref_on_free_pct"] == 100.0, r  # constant map->odom: no jumps
        assert abs(r["amcl_std_xy_m"]["median"] - np.sqrt(0.02)) < 1e-9, r
        assert r["amcl_covers_run_pct"] > 95, r
        c = pe.ref_vs_map(f"{d}/ref.txt", f"{d}/map.yaml")  # what `pose_eval.py check` prints, without AMCL
        assert c["ref_poses"] == 300 and c["ref_on_free_pct"] == 100.0 and abs(c["sensor_height_m"] - 0.5) < 1e-9, c  # RZ90 lifts z by 0.5
        json.dumps(r)


def test_report_flags_amcl_poses_that_stop_before_the_run_ends():
    with tempfile.TemporaryDirectory() as d:
        scene(d, 254)
        rows = open(f"{d}/amcl.tum").read().splitlines()
        open(f"{d}/amcl.tum", "w").write("\n".join(rows[: len(rows) // 3]) + "\n")  # the recorder died / AMCL went silent after a third of the run
        r = pe.report(f"{d}/amcl.tum", f"{d}/odom.tum", f"{d}/ref.txt", f"{d}/map.yaml")
        assert r["amcl_covers_run_pct"] < 50, r


def test_reference_inside_walls_is_flagged():
    with tempfile.TemporaryDirectory() as d:  # map and poses from different runs show up as a trajectory through occupied cells
        r = scene(d, 0)
        assert r["ref_on_occupied_pct"] == 100.0 and r["ref_on_free_pct"] == 0.0, r


def test_run_scripts_only_pass_flags_the_pose_eval_subcommands_accept():
    here = os.path.dirname(os.path.abspath(__file__))
    scripts = sorted(f for f in os.listdir(here) if f.startswith("run_") and f.endswith(".sh"))
    assert {"run_bench_a.sh", "run_amcl_only.sh"} <= set(scripts), scripts
    subs = pe.parser()._subparsers._group_actions[0].choices
    for name in scripts:
        calls = re.findall(r"pose_eval\.py\"? (initpose|init|level|check|report)\b([^\n|]*)", open(os.path.join(here, name)).read())
        assert calls, f"{name} never calls pose_eval.py"
        text = open(os.path.join(here, name)).read()
        if "set_initial_pose=false" in text:  # it switches the parameter off: it must publish the pose itself, or AMCL never starts
            assert any(c == "initpose" for c, _ in calls) and "/initialpose" in text, f"{name} turns set_initial_pose off but never publishes /initialpose"
        for cmd, rest in calls:
            for flag in re.findall(r"(--[a-z-]+)", rest):
                assert flag in subs[cmd]._option_string_actions, f"{name} passes {flag} to `pose_eval.py {cmd}`, which does not take it"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
