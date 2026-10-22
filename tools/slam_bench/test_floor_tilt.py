"""Checks floor_tilt.py on synthetic GLIM submap dumps with a known floor/ceiling tilt. Run: python3 test_floor_tilt.py"""
import tempfile
from pathlib import Path

import numpy as np

import floor_tilt

RNG = np.random.default_rng(1)


def plane_points(n, sx, sy, h, half=10.0):
    """n points on z = sx*x + sy*y + h; 1 cm noise. Returns points and the unit normal (z > 0)."""
    xy = RNG.uniform(-half, half, (n, 2))
    z = sx * xy[:, 0] + sy * xy[:, 1] + h + RNG.normal(0, 0.01, n)
    normal = np.array([-sx, -sy, 1.0]) / np.sqrt(1 + sx**2 + sy**2)
    return np.column_stack([xy, z]), normal


def wall_points(n):
    y = RNG.uniform(-10, 10, n)
    z = RNG.uniform(0, 2.5, n)
    return np.column_stack([np.full(n, 10.0), y, z]), np.tile([-1.0, 0, 0], (n, 1))


def write_submap(d, floor, ceiling, T):
    """Dump one submap in GLIM's layout: points/normals are float32 in the submap origin frame."""
    pts, nrm = [], []
    for pl, sign in ((floor, 1), (ceiling, -1)):
        if pl is None:
            continue
        p, n = plane_points(*pl)
        pts.append(p)
        nrm.append(np.tile(sign * n, (len(p), 1)) + RNG.normal(0, 0.02, (len(p), 3)))  # normals face the sensor
    p, n = wall_points(2000)
    pts.append(p)
    nrm.append(n)
    pts.append(RNG.uniform(-10, 10, (300, 3)))  # outliers
    nrm.append(RNG.normal(0, 1, (300, 3)))
    pw, nw = np.vstack(pts), np.vstack(nrm)
    nw /= np.linalg.norm(nw, axis=1, keepdims=True)
    R, t = T[:3, :3], T[:3, 3]
    d.mkdir()
    (d / "data.txt").write_text("id: 0\nT_world_origin: \n" + "\n".join(" ".join(f"{v:.9g}" for v in row) for row in T) + "\nT_origin_endpoint_L: \n")
    ((pw - t) @ R).astype(np.float32).tofile(d / "points_compact.bin")
    (nw @ R).astype(np.float32).tofile(d / "normals_compact.bin")


def yaw_pose(yaw_deg, x, y):
    a = np.radians(yaw_deg)
    T = np.eye(4)
    T[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    T[:2, 3] = [x, y]
    return T


def test_recovers_known_tilt_and_height():
    sx, sy = np.tan(np.radians(1.5)), np.tan(np.radians(-0.8))
    with tempfile.TemporaryDirectory() as tmp:
        run = Path(tmp)
        write_submap(run / "000000", (6000, sx, sy, 0.0), (6000, sx, sy, 2.5), yaw_pose(37, 3.0, -2.0))
        res = floor_tilt.analyse(run)[0]
    f, c = res["floor"], res["ceiling"]
    assert abs(f["slope_x"] - 1.5) < 0.1 and abs(f["slope_y"] + 0.8) < 0.1, f
    assert abs(c["slope_x"] - 1.5) < 0.1 and abs(c["slope_y"] + 0.8) < 0.1, c
    assert abs(f["tilt"] - np.degrees(np.arctan(np.hypot(sx, sy)))) < 0.1, f
    assert abs(f["z"] - (sx * 3.0 + sy * -2.0)) < 0.03, f  # floor height under the submap origin
    assert abs(c["z"] - (2.5 + sx * 3.0 + sy * -2.0)) < 0.03, c


def test_missing_floor_is_none_not_a_wrong_number():
    with tempfile.TemporaryDirectory() as tmp:
        run = Path(tmp)
        write_submap(run / "000000", (40, 0.0, 0.0, 0.0), (6000, 0.0, 0.0, 2.5), yaw_pose(0, 0, 0))  # only 40 floor points
        res = floor_tilt.analyse(run)[0]
    assert res["floor"] is None and res["ceiling"] is not None


def test_summary_reports_drift_against_first_submap():
    with tempfile.TemporaryDirectory() as tmp:
        run = Path(tmp)
        for i, deg in enumerate((0.0, 1.0, 2.0)):
            s = np.tan(np.radians(deg))
            write_submap(run / f"{i:06d}", (6000, s, 0.0, 0.0), (6000, s, 0.0, 2.5), yaw_pose(10 * i, i, 0))
        s = floor_tilt.summary(floor_tilt.analyse(run))
    assert s["n_submaps"] == 3 and s["floor_n"] == 3
    assert abs(s["floor_drift_max"] - 2.0) < 0.15, s  # 2 deg away from the first submap's floor


def consistent_floor_run(tmp, z_steps):
    """Three submaps at different places whose floors lie on one plane tilted 1 deg about y; z_steps shifts each submap's floor."""
    s = np.tan(np.radians(1.0))
    run = Path(tmp)
    for i, dz in enumerate(z_steps):
        write_submap(run / f"{i:06d}", (6000, s, 0.0, dz), (6000, s, 0.0, 2.5), yaw_pose(20 * i, 15.0 * i, 0))
    return floor_tilt.summary(floor_tilt.analyse(run))


def test_one_tilted_plane_is_flat_but_tilted():
    with tempfile.TemporaryDirectory() as tmp:
        s = consistent_floor_run(tmp, (0.0, 0.0, 0.0))
    assert abs(s["floor_global_slope"] - 1.0) < 0.1, s  # the whole floor is one plane, tilted 1 deg from the world horizontal
    assert s["floor_resid_std"] < 0.02, s               # and flat: ~1 cm noise only


def test_a_floor_step_between_submaps_shows_as_residual():
    with tempfile.TemporaryDirectory() as tmp:
        s = consistent_floor_run(tmp, (0.0, 0.15, 0.0))
    assert s["floor_resid_std"] > 0.05 and s["floor_resid_range"] > 0.1, s


if __name__ == "__main__":
    test_recovers_known_tilt_and_height()
    test_missing_floor_is_none_not_a_wrong_number()
    test_summary_reports_drift_against_first_submap()
    test_one_tilted_plane_is_flat_but_tilted()
    test_a_floor_step_between_submaps_shows_as_residual()
    print("ok")
